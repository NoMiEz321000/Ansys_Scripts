%% ATR 42 / ATR 72  PRELIMINARY V-n DIAGRAM  (JAR-25 / CS-25 style, sea level, EAS)
%  SAVE THIS FILE AS:  atr_vn_diagram.m   (one script per file)
%  Maneuver envelope + discrete-gust envelope + combined envelope.
%  Exports the critical load cases to vn_load_cases.csv for the tail-load step.
%
%  >>> Items marked [ASSUMED] are NOT from ATR/EASA. Replace them with
%  >>> Airplane Flight Manual / ATR data when you have it.
clear; clc; close all;

%% ------------------------- INPUTS --------------------------------------
% --- SELECT AIRCRAFT ---
aircraft = 'ATR42-600';    % 'ATR42-600' (group model) or 'ATR72-600'
n_max_override = [];       % [] = CS-25.337 formula; or force e.g. 2.5

switch aircraft
    case 'ATR42-600'       % source: group's "Parameter Summarize ATR 42" PDF
        W_kg = 18600;  S = 54.5;  b = 24.57;  AR = 11.08;  cbar = 2.285;  % MTOW, wing, MAC
        VMO_kt = 250;  VA_hbk_kt = NaN;
        CLmax_pos = 1.5;   % PDF: clean CLmax "generally around 1.5"
    case 'ATR72-600'       % ATR factsheet, HAW Hamburg study, pilot handbook
        W_kg = 22800;  S = 61.0;  b = 27.166;  AR = 12;  cbar = S/b;
        VMO_kt = 250;  VA_hbk_kt = 175;
        CLmax_pos = 1.6;   % [ASSUMED]
    otherwise
        error('Unknown aircraft: %s', aircraft);
end

% --- [ASSUMED] (both aircraft) ---
CLmax_neg = -1.0;     % clean-wing min CL
VC_kt     = VMO_kt;   % design cruise speed (EAS ~ IAS at sea level)
VD_factor = 1.25;     % VD = 1.25*VC (CS-25.335 minimum margin)
eta       = 0.95;     % airfoil efficiency for lift-curve slope

% Gust velocities (EAS), JAR-25 ch.15 / FAR 25.341 pre-Amdt 86 form  [verify]
Ude_B = 66 * 0.3048;  % 20.12 m/s at VB
Ude_C = 50 * 0.3048;  % 15.24 m/s at VC
Ude_D = 25 * 0.3048;  %  7.62 m/s at VD

%% ------------------------- CONSTANTS -----------------------------------
g = 9.80665;  rho0 = 1.225;  kt2ms = 0.514444;  kg2lb = 2.20462;
W = W_kg * g;                 % weight [N]
WS = W / S;                   % wing loading [N/m^2]

%% ------------------------- DERIVED -------------------------------------
% Lift-curve slope (Helmbold/DATCOM-type, unswept, incompressible) [1/rad]
a = 2*pi*AR / (2 + sqrt(4 + (AR/eta)^2));

% Load-factor limits  (CS-25.337: n = 2.1 + 24000/(W_lb+10000), 2.5 <= n <= 3.8)
n_max = min(max(2.1 + 24000/(W_kg*kg2lb + 10000), 2.5), 3.8);
if ~isempty(n_max_override), n_max = n_max_override; end
n_min = -1.0;

% Speeds [m/s EAS]
VC = VC_kt * kt2ms;
VD = VD_factor * VC;
VS1    = sqrt(2*W / (rho0*S*CLmax_pos));    % 1-g stall, positive
VSneg  = sqrt(2*W / (rho0*S*abs(CLmax_neg)));
VA = VS1   * sqrt(n_max);                   % intersection, positive stall / n_max
VG = VSneg * sqrt(abs(n_min));              % intersection, negative stall / n_min

% Gust alleviation factor (Pratt) and mass ratio
mu = 2*WS / (rho0 * cbar * g * a);
Kg = 0.88*mu / (5.3 + mu);

% VB: design speed for max gust intensity (CS-25.335(d)), capped at VC
VB = min(VS1 * sqrt(1 + Kg*Ude_C*VC*a*rho0*S/(2*W)), VC);

% Gust load-factor increments  dn = Kg*rho0*Ude*V*a*S/(2W)
dn = @(Ude, V) Kg*rho0*Ude.*V*a*S/(2*W);
dnB = dn(Ude_B, VB);  dnC = dn(Ude_C, VC);  dnD = dn(Ude_D, VD);

%% ------------------------- ENVELOPES -----------------------------------
V = linspace(0, VD, 2000);

stall_pos = 0.5*rho0*V.^2*S*CLmax_pos / W;          % +n stall parabola
stall_neg = 0.5*rho0*V.^2*S*CLmax_neg / W;          % -n stall parabola

% Maneuver envelope
man_up  = min(stall_pos, n_max);
man_low = max(stall_neg, n_min);
man_low(V > VC) = n_min * (VD - V(V > VC)) / (VD - VC);   % -1 at VC -> 0 at VD

% Gust lines: (0,1) -> VB -> VC -> VD
gV  = [0 VB VC VD];
gUp = [1, 1+dnB, 1+dnC, 1+dnD];
gLo = [1, 1-dnB, 1-dnC, 1-dnD];
gust_up  = interp1(gV, gUp, V);
gust_low = interp1(gV, gLo, V);

% Combined envelope (gust points above the stall curve are unreachable)
env_up  = max(man_up,  min(gust_up,  stall_pos));
env_low = min(man_low, max(gust_low, stall_neg));

%% ------------------------- CRITICAL POINTS -----------------------------
% name, V [m/s], n, description
pts = {
 'A',     VA, n_max,        'Pos. stall meets n_max (maneuver)';
 'C',     VC, n_max,        'Cruise speed at n_max';
 'D',     VD, n_max,        'Dive speed at n_max';
 'E',     VD, 0,            'Dive speed at n = 0';
 'F',     VC, n_min,        'Cruise speed at n_min';
 'G',     VG, n_min,        'Neg. stall meets n_min';
 'GustB+',VB, 1+dnB,        'Gust +66 ft/s at VB';
 'GustC+',VC, 1+dnC,        'Gust +50 ft/s at VC';
 'GustD+',VD, 1+dnD,        'Gust +25 ft/s at VD';
 'GustB-',VB, 1-dnB,        'Gust -66 ft/s at VB';
 'GustC-',VC, 1-dnC,        'Gust -50 ft/s at VC';
 'GustD-',VD, 1-dnD,        'Gust -25 ft/s at VD'};

%% ------------------------- REPORT --------------------------------------
yesno = {'no','yes'};
gust_inside = max(1+[dnB dnC dnD]) <= n_max && min(1-[dnB dnC dnD]) >= n_min;

fprintf('\n=== %s preliminary V-n (sea level, EAS) ===\n', aircraft);
fprintf('W = %.0f kg, S = %.1f m^2, W/S = %.0f N/m^2, cbar = %.3f m\n', W_kg, S, WS, cbar);
fprintf('a = %.3f /rad, mu = %.1f, Kg = %.3f\n', a, mu, Kg);
fprintf('n_max = %.2f, n_min = %.2f\n', n_max, n_min);
fprintf('VS1 = %.1f kt, VA = %.1f kt (handbook VA = %g kt), VB = %.1f kt\n', ...
        VS1/kt2ms, VA/kt2ms, VA_hbk_kt, VB/kt2ms);
fprintf('VC = %.1f kt, VD = %.1f kt\n\n', VC/kt2ms, VD/kt2ms);
fprintf('%-8s %9s %8s   %s\n', 'Point', 'V [kt]', 'n', 'Description');
for k = 1:size(pts,1)
    fprintf('%-8s %9.1f %8.3f   %s\n', pts{k,1}, pts{k,2}/kt2ms, pts{k,3}, pts{k,4});
end
fprintf('\nGust lines stay inside the maneuver envelope? %s\n', yesno{1 + gust_inside});

%% ------------------------- EXPORT --------------------------------------
fid = fopen('vn_load_cases.csv', 'w');
if fid < 0, error('Cannot write vn_load_cases.csv - check that the current folder is writable.'); end
fprintf(fid, 'point,V_kt_EAS,V_ms_EAS,n,weight_kg,description\n');
for k = 1:size(pts,1)
    fprintf(fid, '%s,%.2f,%.2f,%.4f,%.0f,%s\n', pts{k,1}, pts{k,2}/kt2ms, ...
            pts{k,2}, pts{k,3}, W_kg, pts{k,4});
end
fclose(fid);
fprintf('Saved %s\n', fullfile(pwd, 'vn_load_cases.csv'));

%% ------------------------- PLOT ----------------------------------------
Vk = V / kt2ms;
fig = figure('Color','w','Position',[100 100 980 640]);
try, theme(fig, 'light'); catch, end      % R2025a+: force light theme (ignored on older versions)
hold on; grid on; box on;

% These 5 lines are the legend entries (in this order)
plot(Vk, stall_pos, ':', 'Color',[.6 .6 .6]);
plot(Vk, stall_neg, ':', 'Color',[.6 .6 .6]);
plot(gV/kt2ms, gUp, '--', 'Color',[0.85 0.33 0.10], 'LineWidth',1.2);
plot(gV/kt2ms, gLo, '--', 'Color',[0.85 0.33 0.10], 'LineWidth',1.2);
Venv = [Vk, fliplr(Vk), Vk(1)];
Nenv = [env_up, fliplr(env_low), env_up(1)];
plot(Venv, Nenv, 'k-', 'LineWidth', 2.2);

xlim([0, VD/kt2ms*1.08]); ylim([n_min-0.6, n_max+0.6]);

% Reference lines (kept out of the legend)
yl = ylim;  xl = xlim;
plot([VC VC]/kt2ms, yl, ':k', 'HandleVisibility','off');
plot([VD VD]/kt2ms, yl, ':k', 'HandleVisibility','off');
plot(xl, [0 0], '-', 'Color',[.4 .4 .4], 'HandleVisibility','off');

% Corner points A, C, D, E, F, G
for k = 1:6
    plot(pts{k,2}/kt2ms, pts{k,3}, 'ko', 'MarkerFaceColor','k', 'MarkerSize',5, ...
         'HandleVisibility','off');
    text(pts{k,2}/kt2ms+3, pts{k,3}+0.07, pts{k,1}, 'FontWeight','bold', 'Color','k');
end

xlabel('Equivalent airspeed  V_{EAS}  [kt]');
ylabel('Load factor  n  [-]');
title(sprintf('%s preliminary V-n diagram  (W = %.0f kg, sea level)', aircraft, W_kg));
lg = legend({'Stall (+)','Stall (-)','Gust (+)','Gust (-)','Design envelope'}, 'Location','northwest');
text(5, n_min-0.45, 'PRELIMINARY: CL_{max}, V_C and V_D are assumed', 'Color',[.7 0 0]);

% Explicit light styling so the plot is readable in dark-mode MATLAB too
ax = gca;
set(ax, 'Color','w', 'XColor','k', 'YColor','k', ...
        'GridColor',[0.15 0.15 0.15], 'GridAlpha',0.25, 'FontSize',12);
ax.Title.Color  = 'k';
ax.XLabel.Color = 'k';
ax.YLabel.Color = 'k';
set(lg, 'TextColor','k', 'Color','w', 'EdgeColor',[0.5 0.5 0.5]);

try
    exportgraphics(fig, 'atr_vn_diagram.png', 'Resolution', 200, 'BackgroundColor','white');  % R2020a+
catch
    saveas(fig, 'atr_vn_diagram.png');                                                         % older MATLAB
end
fprintf('Saved %s\n', fullfile(pwd, 'atr_vn_diagram.png'));
