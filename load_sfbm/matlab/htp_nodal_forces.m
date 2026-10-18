%% STEP 4: EXTERNAL NODAL FORCES FOR THE ANSYS APDL MODEL (HORIZONTAL TAIL, ONE HALF)
%  SAVE THIS FILE AS:  atr_htp_nodal_forces.m
%  Input : htp_sfbm.csv   (from atr_htp_sfbm.m: V and M at the rib stations)
%          htp_nodes.csv  (optional: the loaded FE nodes, columns  node,y   y in metres, 0 = root)
%  Output: htp_loads_<case>.inp  (APDL F commands) and htp_nodal_forces.csv
%
%  Method (Palumbo-style): the nodal force is a polynomial in span position,
%        w(y) = sum_p a_p (y/s)^p, integrated over each station's tributary length
%        and split equally over the nodes sharing that y,
%  and the coefficients a are found with lsqminnorm so that the nodal forces
%  reproduce V(y) and M(y) at every cut:
%        V(c) = sum_{y_j > c} F_j            M(c) = sum_{y_j > c} F_j (y_j - c)
%  Sign: F positive = global +Z (up).  Tail load down -> negative forces.
clear; clc; close all;

%% ------------------------- INPUTS --------------------------------------
case_to_export = 'D_fwd';     % case name as in htp_sfbm.csv ('D_fwd' = critical download, 'C_fwd', 'A_aft')
deg      = 3;                 % polynomial degree of the force distribution (unknowns = deg+1)
w_root   = 100;               % extra weight on the root cut (total load and root moment must be exact)
tol_y    = 1e-3;              % nodes closer than this in y [m] are at the same station
apdl_dir = 'FZ';              % force direction label in ANSYS
y_to_m   = 1e-3;              % htp_nodes.csv has y in mm (your APDL model) -> metres

if ~exist('htp_sfbm.csv','file'), error('Run atr_htp_sfbm.m first.'); end
S = readtable('htp_sfbm.csv', 'Delimiter', ',');
S.Properties.VariableNames = {'case_id','station','y_m','w_N_per_m','V_N','M_Nm'};
sel = strcmp(cellstr(S.case_id), case_to_export);
if ~any(sel)
    error('Case "%s" not in htp_sfbm.csv. Available: %s', case_to_export, ...
          strjoin(unique(cellstr(S.case_id))', ', '));
end
yr = S.y_m(sel);  Vr = S.V_N(sel);  Mr = S.M_Nm(sel);
s  = max(yr);

% Loaded nodes ------------------------------------------------------------
if exist('htp_nodes.csv','file')
    Nd = readtable('htp_nodes.csv', 'Delimiter', ',');
    Nd.Properties.VariableNames(1:2) = {'node','y'};
    node_id = Nd.node;  node_y = Nd.y * y_to_m;   % ANSYS model is in mm -> metres
    fprintf('Using %d nodes from htp_nodes.csv\n', numel(node_id));
else
    % DEMO: 2 nodes per rib (front and rear spar). Replace with your real nodes.
    node_y  = repelem(yr, 2);
    node_id = (1:numel(node_y))';
    fprintf('htp_nodes.csv not found -> DEMO nodes (2 per rib station). Export your own, see header.\n');
end
if max(node_y) > s*1.02 || min(node_y) < -tol_y
    warning('Node y range [%.3f, %.3f] m does not match the half-span 0..%.3f m. Check axis/units.', ...
            min(node_y), max(node_y), s);
end

%% ------------------------- STATIONS AND CUTS ---------------------------
[yg, ~, gi] = uniquetol(node_y, tol_y/s);        % unique stations
ng   = numel(yg);
cnt  = accumarray(gi, 1);                        % nodes per station
% Tributary intervals: station g carries the load between the mid-points to its
% neighbours; the first one reaches the root (y = 0) and the last one the tip.
edges  = [0; 0.5*(yg(1:end-1) + yg(2:end)); max(s, yg(end))];
% Cuts: just inboard of the first loaded station (= root), then the mid-bay edges
cutsel = [yg(1) - 10*tol_y; edges(2:end-1)];   % used to choose the outboard stations
cutref = [0;                edges(2:end-1)];   % moment reference position
Vc = interp1(yr, Vr, cutref, 'pchip', 'extrap');
Mc = interp1(yr, Mr, cutref, 'pchip', 'extrap');
Vc(1) = Vr(1);  Mc(1) = Mr(1);                 % exact root values

%% ------------------------- BUILD AND SOLVE -----------------------------
% Distributed load w(y) = sum_p a_p (y/s)^p  [N/m]; the force on each station is
% the integral of w over its tributary interval, shared equally by its nodes.
np  = deg + 1;
pw  = 0:deg;
Phi = s * ((edges(2:end)/s).^(pw+1) - (edges(1:end-1)/s).^(pw+1)) ./ (pw+1);  % station totals
A_V = zeros(numel(cutsel), np);  A_M = A_V;
for k = 1:numel(cutsel)
    out = yg > cutsel(k);                        % stations outboard of the cut
    A_V(k,:) = sum(Phi(out,:), 1);
    A_M(k,:) = sum(Phi(out,:) .* (yg(out) - cutref(k)), 1);
end
wV = ones(numel(cutsel),1);  wV(1) = w_root;
wM = wV;
sV = max(abs(Vc));  sM = max(abs(Mc));           % normalise so V and M rows are comparable
A = [A_V .* (wV/sV); A_M .* (wM/sM)];
b = [Vc .* (wV/sV); Mc .* (wM/sM)];
a = lsqminnorm(A, b);

Ftot  = Phi * a;                                 % total force at each station
Fstat = Ftot ./ cnt;                             % force on EACH node of the station
F = Fstat(gi);                                   % force per node (in node order)

%% ------------------------- CHECK ---------------------------------------
Vn = zeros(size(yr));  Mn = Vn;
for k = 1:numel(yr)
    out = node_y > yr(k) - tol_y;                % nodes at or outboard of the rib station
    Vn(k) = sum(F(out));  Mn(k) = sum(F(out) .* (node_y(out) - yr(k)));
end
fprintf('\nCase %s : sum of nodal forces = %.1f N (target V_root = %.1f N)\n', case_to_export, sum(F), Vr(1));
fprintf('Root moment from nodal forces = %.1f N m (target %.1f N m)\n', Mn(1), Mr(1));
fprintf('Max |V error| at ribs = %.1f N (%.1f%% of root),  max |M error| = %.1f N m (%.1f%% of root)\n', ...
        max(abs(Vn - Vr)), 100*max(abs(Vn - Vr))/abs(Vr(1)), max(abs(Mn - Mr)), 100*max(abs(Mn - Mr))/abs(Mr(1)));
fprintf('(V at a rib itself includes that rib''s own point load, so it differs by up to half a station force;\n');
fprintf(' the cuts mid-way between ribs are matched exactly.)\n');

%% ------------------------- EXPORT --------------------------------------
fid = fopen(sprintf('htp_loads_%s.inp', case_to_export), 'w');
fprintf(fid, '! Horizontal-tail external nodal forces, case %s\n', case_to_export);
fprintf(fid, '! Generated by atr_htp_nodal_forces.m   (+ = global +Z)\n');
for j = 1:numel(F)
    fprintf(fid, 'F,%d,%s,%.4f\n', node_id(j), apdl_dir, F(j));
end
fclose(fid);
Tout = table(node_id(:), node_y(:), F(:), 'VariableNames', {'node','y_m','F_N'});
writetable(Tout, 'htp_nodal_forces.csv');
fprintf('Saved htp_loads_%s.inp and htp_nodal_forces.csv\n', case_to_export);

%% ------------------------- PLOT ----------------------------------------
fig = figure('Color','w','Position',[80 80 1100 430]);
try, theme(fig,'light'); catch, end
subplot(1,3,1); stem(yg, Ftot, 'filled', 'Color',[0 0.45 0.74]); grid on;
title('Force per station [N]'); xlabel('y [m]');
subplot(1,3,2); plot(yr, Vr, 'k-', yr, Vn, 'o', 'LineWidth',1.5); grid on;
title('Shear V(y) [N]'); xlabel('y [m]'); legend('target','from nodal forces','Location','best');
subplot(1,3,3); plot(yr, Mr, 'k-', yr, Mn, 'o', 'LineWidth',1.5); grid on;
title('Moment M(y) [N m]'); xlabel('y [m]');
for ax = findall(fig,'Type','axes')'
    set(ax, 'Color','w','XColor','k','YColor','k'); ax.Title.Color = 'k'; ax.XLabel.Color = 'k';
end
try
    exportgraphics(fig, 'atr_htp_nodal_forces.png', 'Resolution', 200, 'BackgroundColor','white');
catch
    saveas(fig, 'atr_htp_nodal_forces.png');
end