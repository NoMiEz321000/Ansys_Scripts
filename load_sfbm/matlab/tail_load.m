%% ATR 42 / ATR 72  -  STEP 2: TOTAL (BALANCING) HORIZONTAL-TAIL LOAD AT THE V-n POINTS
%  Reads vn_load_cases.csv (made by atr_vn_diagram.m) and computes the tail
%  load L_H [N] needed to hold pitch equilibrium at each V-n point, for the
%  forward and aft CG limits.   Sign: L_H > 0 = tail lift UP, < 0 = DOWN.
%
%  Equilibrium (about the CG, steady, small angles, thrust/drag couples ignored):
%      L_w + L_H = n W                                  (vertical force)
%      L_w*(xcg - xac)*cbar + M_ac - L_H*l_H = 0        (moments, nose-up +)
%  ->  L_H = ( n W (xcg-xac) cbar + M_ac ) / ( l_H + (xcg-xac) cbar )
%      M_ac = Cm0 * q * S * cbar     (wing-body zero-lift moment, nose-down < 0)
%
%  >>> Items marked [ASSUMED] are NOT confirmed aircraft data. Replace them
%  >>> with Airplane Flight Manual / ATR data when you have it.
clear; clc; close all;

%% ------------------------- INPUTS --------------------------------------
if ~exist('vn_load_cases.csv','file')
    error('Run atr_vn_diagram.m first - it creates vn_load_cases.csv.');
end
T = readtable('vn_load_cases.csv', 'Delimiter', ',');
if width(T) ~= 6
    error(['vn_load_cases.csv should have 6 columns but has %d (%s). ' ...
           'Delete it, re-run atr_vn_diagram.m, and do not re-save it from Excel.'], ...
           width(T), strjoin(T.Properties.VariableNames, ', '));
end
T.Properties.VariableNames = {'point','V_kt_EAS','V_ms_EAS','n','weight_kg','description'};
if height(T) ~= 12
    error(['vn_load_cases.csv should have 12 rows (V-n points) but %d were read. ' ...
           'Delete it and re-run atr_vn_diagram.m.'], height(T));
end

% --- SELECT AIRCRAFT (must match the one used in atr_vn_diagram.m) ---
aircraft = 'ATR42-600';    % 'ATR42-600' (group model) or 'ATR72-600'

switch aircraft
    case 'ATR42-600'       % source: group's "Parameter Summarize ATR 42" PDF
        S = 54.5;  b = 24.57;  cbar = 2.285;   % wing area, span, MAC [m]
        l_H     = 11.46;   % CG -> tail aerodynamic centre [m]
        S_H     = 19.69;   % HTP total area [m^2] from Palumbo's tail: 2 x 0.5 x (3.090+1.860) x 3.978
                           % (only used for the CL_H sanity check; L_H itself does not depend on S_H)
        xcg_fwd = 0.168;   % forward CG limit, fraction of MAC   (PDF)
        xcg_aft = 0.325;   % aft CG limit                        (PDF)
        CLmax_w = 1.5;     % clean CLmax                         (PDF, same as V-n script)
        MTOW_expected = 18600;
    case 'ATR72-600'
        S = 61.0;  b = 27.166;  cbar = S/b;    % [ASSUMED] MAC ~ S/b
        l_H     = 13.565;  % [ASSUMED] HAW conceptual study
        S_H     = 9.7;     % [ASSUMED] HAW conceptual study
        xcg_fwd = 0.168;   % [ASSUMED] ATR 42 value
        xcg_aft = 0.325;   % [ASSUMED] ATR 42 value
        CLmax_w = 1.6;     % [ASSUMED]
        MTOW_expected = 22800;
    otherwise
        error('Unknown aircraft: %s', aircraft);
end

% --- Longitudinal aerodynamics (both aircraft) ---
xac      = 0.25;    % wing-body aerodynamic centre, fraction of MAC (standard 25%)
Cm0_ref  = -0.06;   % [ASSUMED] wing-body zero-lift moment coeff., flaps up
Cm0_list = [-0.03 -0.06 -0.09];   % sensitivity sweep

%% ------------------------- CONSTANTS -----------------------------------
g = 9.80665;  rho0 = 1.225;
W_kg = T.weight_kg(1);   W = W_kg * g;           % weight from the V-n run [N]
if abs(W_kg - MTOW_expected) > 1
    warning('CSV weight (%.0f kg) does not match %s MTOW (%.0f kg): re-run the V-n script for this aircraft.', W_kg, aircraft, MTOW_expected);
end
q  = 0.5 * rho0 * T.V_ms_EAS.^2;                 % dynamic pressure [Pa] (EAS, sea level)
n  = T.n;
pt = T.point;  N = height(T);

% Balancing tail load [N]; vectorised in n and q
LH = @(n, q, xcg, Cm0) (n*W*(xcg-xac)*cbar + Cm0*q*S*cbar) ./ (l_H + (xcg-xac)*cbar);

%% ------------------------- CALCULATE -----------------------------------
LH_f = LH(n, q, xcg_fwd, Cm0_ref);     % forward CG
LH_a = LH(n, q, xcg_aft, Cm0_ref);     % aft CG
CLw_f = (n*W - LH_f) ./ (q*S);         % wing CL needed (checks vs CLmax)
CLw_a = (n*W - LH_a) ./ (q*S);
CLH_f = LH_f ./ (q*S_H);               % tail CL (sanity: should be well below ~1)
CLH_a = LH_a ./ (q*S_H);

%% ------------------------- REPORT --------------------------------------
fprintf('\n=== %s balancing tail load  (W = %.0f kg, Cm0 = %.2f) ===\n', aircraft, W_kg, Cm0_ref);
fprintf('cbar = %.3f m, l_H = %.3f m, S_H = %.1f m^2, xac = %.2f\n', cbar, l_H, S_H, xac);
fprintf('CG fwd = %.1f %%MAC, CG aft = %.1f %%MAC\n\n', 100*xcg_fwd, 100*xcg_aft);
fprintf('%-8s %8s %7s %8s | %10s %7s | %10s %7s\n', ...
        'Point','V [kt]','n','q [Pa]','L_H fwd[N]','CL_H','L_H aft[N]','CL_H');
for k = 1:N
    fprintf('%-8s %8.1f %7.3f %8.0f | %10.0f %7.3f | %10.0f %7.3f\n', ...
        pt{k}, T.V_kt_EAS(k), n(k), q(k), LH_f(k), CLH_f(k), LH_a(k), CLH_a(k));
end

allLH = [LH_f; LH_a];
labels = [strcat(pt, ' / fwd CG'); strcat(pt, ' / aft CG')];
[Lmin, iMin] = min(allLH);  [Lmax, iMax] = max(allLH);
fprintf('\nMost negative (tail DOWN): %.0f N  at %s\n', Lmin, labels{iMin});
fprintf('Most positive (tail UP)  : %.0f N  at %s\n', Lmax, labels{iMax});
if any(CLw_f > CLmax_w*1.001) || any(CLw_a > CLmax_w*1.001)
    fprintf('Note: wing CL slightly exceeds CLmax_w at the stall corner(s) - tail download adds to wing lift.\n');
end
if max(abs([CLH_f; CLH_a])) > 1.0
    fprintf('WARNING: |CL_H| > 1.0 at some point - check S_H, l_H or Cm0.\n');
end

% Sensitivity to the assumed Cm0
fprintf('\nSensitivity to Cm0 (all points, both CG limits):\n');
fprintf('%8s %14s %14s\n', 'Cm0', 'min L_H [N]', 'max L_H [N]');
for c = Cm0_list
    L_all = [LH(n,q,xcg_fwd,c); LH(n,q,xcg_aft,c)];
    fprintf('%8.2f %14.0f %14.0f\n', c, min(L_all), max(L_all));
end
fprintf('\nThese are BALANCING loads only. Sizing also needs the manoeuvre/gust increments\n');
fprintf('(CS-25.331, 25.341, 25.427) and a safety factor of 1.5 for ultimate loads.\n');

%% ------------------------- EXPORT --------------------------------------
fid = fopen('tail_load_cases.csv','w');
fprintf(fid, 'case,point,cg,V_kt_EAS,n,q_Pa,weight_kg,Cm0,L_H_N,CL_H\n');
for k = 1:N
    fprintf(fid, '%s_fwd,%s,fwd,%.2f,%.4f,%.1f,%.0f,%.3f,%.1f,%.4f\n', pt{k}, pt{k}, ...
            T.V_kt_EAS(k), n(k), q(k), W_kg, Cm0_ref, LH_f(k), CLH_f(k));
    fprintf(fid, '%s_aft,%s,aft,%.2f,%.4f,%.1f,%.0f,%.3f,%.1f,%.4f\n', pt{k}, pt{k}, ...
            T.V_kt_EAS(k), n(k), q(k), W_kg, Cm0_ref, LH_a(k), CLH_a(k));
end
fclose(fid);
fprintf('Saved tail_load_cases.csv (input for step 3: spanwise load, shear, moment)\n');

%% ------------------------- PLOT ----------------------------------------
fig = figure('Color','w','Position',[100 100 1000 560]);
try, theme(fig,'light'); catch, end
hb = bar([LH_f LH_a]/1e3, 'grouped');
hb(1).FaceColor = [0.00 0.45 0.74];
hb(2).FaceColor = [0.85 0.33 0.10];
grid on; box on;
set(gca, 'XTick',1:N, 'XTickLabel',pt, 'Color','w', 'XColor','k', 'YColor','k', ...
         'GridColor',[0.15 0.15 0.15], 'GridAlpha',0.25, 'FontSize',12);
xlabel('V-n point'); ylabel('Balancing tail load  L_H  [kN]   (+ up)');
title(sprintf('%s balancing tail load  (W = %.0f kg, C_{m0} = %.2f)', aircraft, W_kg, Cm0_ref));
lg = legend({sprintf('Forward CG (%.1f%% MAC)',100*xcg_fwd), ...
             sprintf('Aft CG (%.1f%% MAC)',100*xcg_aft)}, 'Location','southwest');
set(lg, 'TextColor','k', 'Color','w', 'EdgeColor',[0.5 0.5 0.5]);
ax = gca; ax.Title.Color = 'k'; ax.XLabel.Color = 'k'; ax.YLabel.Color = 'k';
text(0.6, max(ylim)*0.92, 'PRELIMINARY: Cm_0 is assumed; check CG limits, l_H, S_H against ATR data', 'Color',[0.7 0 0]);
try
    exportgraphics(fig, 'atr_tail_load.png', 'Resolution', 200, 'BackgroundColor','white');  % R2020a+
catch
    saveas(fig, 'atr_tail_load.png');                                                         % older MATLAB
end
fprintf('Saved %s\n', fullfile(pwd, 'atr_tail_load.png'));