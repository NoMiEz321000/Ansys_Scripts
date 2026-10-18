%% STEP 3: SPANWISE LOAD, SHEAR FORCE AND BENDING MOMENT ON THE HORIZONTAL TAIL (ONE HALF)
%  SAVE THIS FILE AS:  atr_htp_sfbm.m
%  Reads tail_load_cases.csv (made by atr_tail_load.m). For each selected case it
%  spreads the half-tail load P = L_H/2 along the half-span, then integrates from
%  the tip to the root:
%        V(y) = int_y^s  w dy'          (shear force)
%        M(y) = int_y^s  V dy'          (bending moment)
%  Sign follows L_H:  tail load DOWN  ->  V and M negative.
%  No torsion (not required for this project).
%
%  Loads are VERTICAL in the global frame and the moment arms are horizontal, so
%  the 7 deg dihedral does not change V(y) or M(y). It matters only for the FE geometry.
%
%  Geometry: Palumbo et al. (2026) Table 2.  The tail load itself comes from the
%  ATR 42 balancing analysis (atr_tail_load.m).
clear; clc; close all;

%% ------------------------- INPUTS --------------------------------------
if ~exist('tail_load_cases.csv','file')
    error('Run atr_tail_load.m first - it creates tail_load_cases.csv.');
end
T = readtable('tail_load_cases.csv', 'Delimiter', ',');
if width(T) ~= 10
    error(['tail_load_cases.csv should have 10 columns but has %d (%s). ' ...
           'Delete it, re-run atr_tail_load.m, and do not re-save it from Excel.'], ...
           width(T), strjoin(T.Properties.VariableNames, ', '));
end
T.Properties.VariableNames = {'case_id','point','cg','V_kt_EAS','n','q_Pa', ...
                              'weight_kg','Cm0','L_H_N','CL_H'};
if height(T) ~= 24
    error(['tail_load_cases.csv should have 24 rows (12 points x fwd/aft) but %d were read. ' ...
           'Delete it and re-run atr_vn_diagram.m, then atr_tail_load.m.'], height(T));
end

% Cases to analyse (names are  <V-n point>_<fwd|aft>  as in tail_load_cases.csv)
cases = {'D_fwd', 'C_fwd', 'A_aft'};

% Spanwise load shape:  'chord' | 'elliptical' | 'schrenk' (average of the two)
dist = 'schrenk';

% Horizontal tail geometry (one side), Palumbo et al. 2026, Table 2
Cr = 3.090;       % root chord [m]
Ct = 1.860;       % tip chord  [m]
s  = 3.978;       % half-span (one side only) [m]
dihedral_deg = 7; % [deg]  (information only - see header note)
n_ribs = 14;      % rib stations along the half-span (baseline layout in the proposal)

%% ------------------------- LOAD SHAPES ---------------------------------
y = linspace(0, s, 4001);                  % fine grid, y = 0 at root, y = s at tip
c = Cr - (Cr - Ct) * y / s;                % chord (straight trailing edge, linear taper)

shape_chord = c / trapz(y, c);                         % proportional to chord
shape_ell   = (4/(pi*s)) * sqrt(max(1 - (y/s).^2, 0)); % quarter-ellipse
switch lower(dist)
    case 'chord',      shape = shape_chord;
    case 'elliptical', shape = shape_ell;
    case 'schrenk',    shape = 0.5*(shape_chord + shape_ell);
    otherwise,         error('Unknown distribution: %s', dist);
end
% Every shape integrates to 1 over the half-span, so w = P*shape carries exactly P.
y_cen = trapz(y, y .* shape);              % load centroid (moment arm) [m]

S_half = 0.5*(Cr + Ct) * s;
fprintf('\n=== Horizontal tail, one half ===\n');
fprintf('Cr = %.3f m, Ct = %.3f m, half-span = %.3f m, taper = %.3f\n', Cr, Ct, s, Ct/Cr);
fprintf('Area per half = %.2f m^2, total = %.2f m^2, AR = %.2f\n', S_half, 2*S_half, (2*s)^2/(2*S_half));
fprintf('Load shape: %s, centroid at %.3f m (%.1f%% of half-span)\n\n', dist, y_cen, 100*y_cen/s);

%% ------------------------- CALCULATE -----------------------------------
nc = numel(cases);
Wd = zeros(nc, numel(y));   Vd = Wd;   Md = Wd;
LH = zeros(nc, 1);
for k = 1:nc
    idx = find(strcmp(cellstr(T.case_id), cases{k}), 1);
    if isempty(idx)
        error('Case "%s" not found in tail_load_cases.csv. Available: %s', ...
              cases{k}, strjoin(cellstr(T.case_id)', ', '));
    end
    LH(k) = T.L_H_N(idx);
    P = LH(k) / 2;                              % load carried by ONE half [N]

    w = P * shape;                              % distributed load [N/m]
    v = P - cumtrapz(y, w);                     % shear force: root = P, tip = 0
    m = trapz(y, v) - cumtrapz(y, v);           % bending moment: root = P*y_cen, tip = 0

    Wd(k,:) = w;  Vd(k,:) = v;  Md(k,:) = m;
end

%% ------------------------- REPORT --------------------------------------
fprintf('%-8s %10s %12s %14s %14s\n', 'Case', 'L_H [N]', 'V_root [N]', 'M_root [N m]', 'check P*arm');
for k = 1:nc
    fprintf('%-8s %10.0f %12.0f %14.0f %14.0f\n', cases{k}, LH(k), Vd(k,1), Md(k,1), LH(k)/2*y_cen);
end
fprintf('\nChecks: V_root = L_H/2, M_root = (L_H/2) x centroid arm, V = M = 0 at the tip.\n');
fprintf('Sign: negative = tail load DOWN, positive = tail load UP.\n');

%% ------------------------- EXPORT (rib stations) -----------------------
y_rib = linspace(0, s, n_ribs);
fid = fopen('htp_sfbm.csv', 'w');
if fid < 0, error('Cannot write htp_sfbm.csv - check that the current folder is writable.'); end
fprintf(fid, 'case,station,y_m,w_N_per_m,V_N,M_Nm\n');
for k = 1:nc
    wr = interp1(y, Wd(k,:), y_rib);
    vr = interp1(y, Vd(k,:), y_rib);
    mr = interp1(y, Md(k,:), y_rib);
    for i = 1:n_ribs
        fprintf(fid, '%s,%d,%.4f,%.3f,%.2f,%.2f\n', cases{k}, i, y_rib(i), wr(i), vr(i), mr(i));
    end
end
fclose(fid);
fprintf('Saved %s  (%d rib stations per case; input for the nodal-force solver)\n', ...
        fullfile(pwd, 'htp_sfbm.csv'), n_ribs);

%% ------------------------- PLOT ----------------------------------------
cols = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19; 0.49 0.18 0.56];
data = {Wd, Vd, Md};
ttl  = {'Spanwise load  w(y)  [N/m]', 'Shear force  V(y)  [N]', 'Bending moment  M(y)  [N m]'};
labels = cell(1, nc);
for k = 1:nc
    labels{k} = sprintf('%s  (L_H = %.1f kN)', strrep(cases{k}, '_', ' '), LH(k)/1e3);
end

fig = figure('Color','w','Position',[60 80 1280 470]);
try, theme(fig, 'light'); catch, end
axs = gobjects(1, 3);
for p = 1:3
    axs(p) = subplot(1, 3, p);
    hold on; grid on; box on;
    for k = 1:nc
        plot(y, data{p}(k,:), 'LineWidth', 2, 'Color', cols(mod(k-1, 4)+1, :));
    end
    plot(xlim, [0 0], '-', 'Color', [.4 .4 .4], 'HandleVisibility', 'off');
    xlabel('Half-span position  y  [m]   (0 = root, tip at right)');
    title(ttl{p});
    set(axs(p), 'Color','w', 'XColor','k', 'YColor','k', ...
                'GridColor',[0.15 0.15 0.15], 'GridAlpha',0.25, 'FontSize',11);
    axs(p).Title.Color  = 'k';
    axs(p).XLabel.Color = 'k';
end
lg = legend(axs(1), labels, 'Location', 'best');
set(lg, 'TextColor','k', 'Color','w', 'EdgeColor',[0.5 0.5 0.5]);

try
    exportgraphics(fig, 'atr_htp_sfbm.png', 'Resolution', 200, 'BackgroundColor','white');  % R2020a+
catch
    saveas(fig, 'atr_htp_sfbm.png');                                                          % older MATLAB
end
fprintf('Saved %s\n', fullfile(pwd, 'atr_htp_sfbm.png'));