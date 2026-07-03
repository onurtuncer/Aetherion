% build_f16_simulink.m
%
% Builds an F-16 closed-loop Simulink model from two FMUs:
%
%   F16Plant      — Aetherion 6-DoF plant    (FMI 2.0, F16PlantFMU.cpp)
%   F16Autopilot  — NASA LaRC LQR autopilot  (FMI 2.0, F16AutopilotFMU.cpp
%                   wrapping F16_control.dml by Bruce Jackson, NASA TM-2015-218675)
%
% The autopilot implements full LQR stability augmentation plus outer-loop
% altitude, airspeed (KEAS) and heading hold.  No Simulink PID blocks are used:
% all control logic lives inside the DAVE-ML model loaded by the FMU.
%
% Plant FMU outputs feed directly into autopilot FMU inputs (after unit
% conversion constants).  Autopilot FMU outputs feed directly into plant FMU
% inputs.  A 30 s trim-hold simulation is run and CI assertions verify that
% altitude, airspeed and heading remain bounded near the design trim point.
%
% Environment (set up by simulink.yml):
%   FMIKit-Simulink/            — FMI Kit (CATIA-Systems), repo root
%   matlab/F16Plant.fmu         — plant FMU artifact from build-fmu job
%   matlab/F16Autopilot.fmu     — autopilot FMU artifact from build-fmu job
%
% ─────────────────────────────────────────────────────────────────────────────
% F16Plant FMU output port map (registration order in F16PlantFMU.cpp):
%   1  out.alt_m        4  out.yaw_rad      5  out.pitch_rad
%   6  out.roll_rad     7  out.p_rad_s      8  out.q_rad_s
%   9  out.r_rad_s     13  out.alpha_deg   14  out.beta_deg
%  17  out.vt_m_s      24  out.rho_kg_m3
%
% F16Autopilot FMU input port map (registration order in F16AutopilotFMU.cpp):
%   1  cmd.altCmd_ft    2  cmd.keasCmd_kt  3  cmd.baseChiCmd_deg
%   4  cmd.latOffset_ft
%   5  fb.alt_m         6  fb.vt_m_s       7  fb.rho_kg_m3
%   8  fb.alpha_deg     9  fb.beta_deg    10  fb.roll_rad
%  11  fb.pitch_rad    12  fb.yaw_rad     13  fb.p_rad_s
%  14  fb.q_rad_s      15  fb.r_rad_s
%
% F16Autopilot FMU output port map:
%   1  ctrl.el_deg      2  ctrl.ail_deg    3  ctrl.rdr_deg
%   4  ctrl.pwr_pct
% ─────────────────────────────────────────────────────────────────────────────

%% Paths
% MATLAB's run() temporarily cd's into this script's own folder while it
% executes, so repoRoot must be derived from the script location rather
% than pwd (which would resolve to .../matlab, not the repo root).
scriptDir  = fileparts(mfilename('fullpath'));
repoRoot   = fileparts(scriptDir);
resultsDir = fullfile(repoRoot, 'matlab', 'results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

plantFmu  = fullfile(repoRoot, 'matlab', 'F16Plant.fmu');
apFmu     = fullfile(repoRoot, 'matlab', 'F16Autopilot.fmu');

%% FMI Kit setup
fmiKitDir = fullfile(repoRoot, 'FMIKit-Simulink');
if ~isfolder(fullfile(fmiKitDir, '+FMIKit'))
    entries = dir(repoRoot);
    error('build_f16_simulink:FMIKitNotFound', ...
        ['+FMIKit package not found at "%s".\nrepoRoot = %s\n' ...
         'Contents of repoRoot: %s'], ...
        fullfile(fmiKitDir, '+FMIKit'), repoRoot, ...
        strjoin({entries.name}, ', '));
end
addpath(fmiKitDir);
rehash toolboxcache;
FMIKit.initialize();

%% Trim-point commands (match F16_control.dml design point / F16PlantFMU defaults)
ALT_CMD_FT   = 10013.0;    % ft  (≈ 3051.6 m)
KEAS_CMD_KT  = 287.809;    % kt  (≈ 172.4 m/s TAS at 10 013 ft ISA)
HDG_CMD_DEG  = 45.0;       % deg (CW from North)
LAT_OFFSET_FT = 0.0;       % ft  (pure heading hold, no lateral offset)

% Reference values for CI assertions (SI)
ALT_REF_M   = ALT_CMD_FT * 0.3048;
VT_REF_MS   = 565.685    * 0.3048;  % trim TAS from F16PlantFMU default

%% Create model
MDL = 'F16_Autopilot';
if bdIsLoaded(MDL), close_system(MDL, 0); end
new_system(MDL);

% Continuous-time, variable-step, 30 s
set_param(MDL, 'Solver','ode45', 'StopTime','30', 'RelTol','1e-4', 'AbsTol','1e-6');

p = @(x,y,w,h) [x, y, x+w, y+h];  % position helper

% ── F16Plant FMU block ──────────────────────────────────────────────────────
PLANT = [MDL '/F16Plant'];
add_block('FMIKit/FMU', PLANT, 'Position', p(480, 80, 160, 360));
FMIKit.loadFMU(PLANT, plantFmu);

% ── F16Autopilot FMU block ──────────────────────────────────────────────────
AP = [MDL '/F16Autopilot'];
add_block('FMIKit/FMU', AP, 'Position', p(130, 80, 160, 310));
FMIKit.loadFMU(AP, apFmu);

% ── Autopilot command constants ─────────────────────────────────────────────
add_block('simulink/Sources/Constant', [MDL '/AltCmd'], ...
    'Value', num2str(ALT_CMD_FT),   'Position', p(30, 55,  65, 25));
add_block('simulink/Sources/Constant', [MDL '/KeasCmd'], ...
    'Value', num2str(KEAS_CMD_KT),  'Position', p(30, 105, 65, 25));
add_block('simulink/Sources/Constant', [MDL '/HdgCmd'], ...
    'Value', num2str(HDG_CMD_DEG),  'Position', p(30, 155, 65, 25));
add_block('simulink/Sources/Constant', [MDL '/LatOffset'], ...
    'Value', num2str(LAT_OFFSET_FT),'Position', p(30, 205, 65, 25));

% ── To-Workspace sinks ──────────────────────────────────────────────────────
add_block('simulink/Sinks/To Workspace', [MDL '/TW_Alt'], ...
    'VariableName','alt_m',   'SaveFormat','Timeseries','Position', p(700, 80,  75, 25));
add_block('simulink/Sinks/To Workspace', [MDL '/TW_VT'], ...
    'VariableName','vt_ms',   'SaveFormat','Timeseries','Position', p(700, 120, 75, 25));
add_block('simulink/Sinks/To Workspace', [MDL '/TW_Hdg'], ...
    'VariableName','hdg_rad', 'SaveFormat','Timeseries','Position', p(700, 160, 75, 25));

% ── Wire: command constants → autopilot FMU inputs ──────────────────────────
% AP input ports: 1=altCmd_ft  2=keasCmd_kt  3=baseChiCmd_deg  4=latOffset_ft
add_line(MDL, 'AltCmd/1',    'F16Autopilot/1', 'autorouting','on');
add_line(MDL, 'KeasCmd/1',   'F16Autopilot/2', 'autorouting','on');
add_line(MDL, 'HdgCmd/1',    'F16Autopilot/3', 'autorouting','on');
add_line(MDL, 'LatOffset/1', 'F16Autopilot/4', 'autorouting','on');

% ── Wire: plant FMU outputs → autopilot FMU feedback inputs ─────────────────
% AP input ports: 5=fb.alt_m  6=fb.vt_m_s  7=fb.rho_kg_m3
%                 8=fb.alpha_deg  9=fb.beta_deg
%                10=fb.roll_rad  11=fb.pitch_rad  12=fb.yaw_rad
%                13=fb.p_rad_s   14=fb.q_rad_s    15=fb.r_rad_s
add_line(MDL, 'F16Plant/1',  'F16Autopilot/5',  'autorouting','on');  % alt_m
add_line(MDL, 'F16Plant/17', 'F16Autopilot/6',  'autorouting','on');  % vt_m_s
add_line(MDL, 'F16Plant/24', 'F16Autopilot/7',  'autorouting','on');  % rho_kg_m3
add_line(MDL, 'F16Plant/13', 'F16Autopilot/8',  'autorouting','on');  % alpha_deg
add_line(MDL, 'F16Plant/14', 'F16Autopilot/9',  'autorouting','on');  % beta_deg
add_line(MDL, 'F16Plant/6',  'F16Autopilot/10', 'autorouting','on');  % roll_rad
add_line(MDL, 'F16Plant/5',  'F16Autopilot/11', 'autorouting','on');  % pitch_rad
add_line(MDL, 'F16Plant/4',  'F16Autopilot/12', 'autorouting','on');  % yaw_rad
add_line(MDL, 'F16Plant/7',  'F16Autopilot/13', 'autorouting','on');  % p_rad_s
add_line(MDL, 'F16Plant/8',  'F16Autopilot/14', 'autorouting','on');  % q_rad_s
add_line(MDL, 'F16Plant/9',  'F16Autopilot/15', 'autorouting','on');  % r_rad_s

% ── Wire: autopilot FMU outputs → plant FMU control inputs ──────────────────
% AP output ports: 1=ctrl.el_deg  2=ctrl.ail_deg  3=ctrl.rdr_deg  4=ctrl.pwr_pct
% Plant input ports: 1=ctrl.el_deg  2=ctrl.ail_deg  3=ctrl.rdr_deg  4=ctrl.pwr_pct
add_line(MDL, 'F16Autopilot/1', 'F16Plant/1', 'autorouting','on');  % el_deg
add_line(MDL, 'F16Autopilot/2', 'F16Plant/2', 'autorouting','on');  % ail_deg
add_line(MDL, 'F16Autopilot/3', 'F16Plant/3', 'autorouting','on');  % rdr_deg
add_line(MDL, 'F16Autopilot/4', 'F16Plant/4', 'autorouting','on');  % pwr_pct

% ── Wire: plant outputs → workspace sinks ───────────────────────────────────
add_line(MDL, 'F16Plant/1',  'TW_Alt/1', 'autorouting','on');   % out.alt_m
add_line(MDL, 'F16Plant/17', 'TW_VT/1',  'autorouting','on');   % out.vt_m_s
add_line(MDL, 'F16Plant/4',  'TW_Hdg/1', 'autorouting','on');   % out.yaw_rad

%% Save model
slxPath = fullfile(repoRoot, 'matlab', 'F16_Autopilot.slx');
save_system(MDL, slxPath);
fprintf('Model saved: %s\n', slxPath);

%% Run simulation
fprintf('Starting 30 s trim-hold simulation (NASA LQR autopilot + F16 plant)...\n');
simOut = sim(MDL);
fprintf('Simulation complete.\n');

%% Extract results
t       = simOut.alt_m.Time;
alt_m   = simOut.alt_m.Data;
vt_ms   = simOut.vt_ms.Data;
hdg_rad = simOut.hdg_rad.Data;
hdg_deg = hdg_rad * (180 / pi);

final_alt = alt_m(end);
final_vt  = vt_ms(end);
final_hdg = hdg_deg(end);

fprintf('Final state — alt: %.2f m | vt: %.2f m/s | hdg: %.2f deg\n', ...
        final_alt, final_vt, final_hdg);

%% Save results
save(fullfile(resultsDir, 'sim_results.mat'), ...
     't', 'alt_m', 'vt_ms', 'hdg_deg', ...
     'ALT_REF_M', 'VT_REF_MS', 'HDG_CMD_DEG');

%% Plot
fig = figure('Visible', 'off');

subplot(3,1,1);
plot(t, alt_m,  'b-', 'LineWidth', 1.2); hold on;
yline(ALT_REF_M, 'r--', 'LineWidth', 1);
ylabel('Altitude [m]'); legend('actual','ref','Location','best'); grid on;
title('F-16 NASA LQR Autopilot — 30 s Trim-Hold Simulation');

subplot(3,1,2);
plot(t, vt_ms, 'b-', 'LineWidth', 1.2); hold on;
yline(VT_REF_MS, 'r--', 'LineWidth', 1);
ylabel('TAS [m/s]'); legend('actual','ref','Location','best'); grid on;

subplot(3,1,3);
plot(t, hdg_deg, 'b-', 'LineWidth', 1.2); hold on;
yline(HDG_CMD_DEG, 'r--', 'LineWidth', 1);
ylabel('Heading [deg]'); xlabel('Time [s]');
legend('actual','ref','Location','best'); grid on;

saveas(fig, fullfile(resultsDir, 'f16_autopilot.png'));
close(fig);
fprintf('Results saved to %s\n', resultsDir);

%% CI assertions — trim hold: states must stay near design trim
assert(abs(final_alt - ALT_REF_M) < 100, ...
    'Altitude hold failed: error = %.1f m (limit 100 m)', final_alt - ALT_REF_M);
assert(abs(final_vt - VT_REF_MS) < 10, ...
    'Airspeed hold failed: error = %.2f m/s (limit 10 m/s)', final_vt - VT_REF_MS);
assert(abs(final_hdg - HDG_CMD_DEG) < 10, ...
    'Heading hold failed: error = %.2f deg (limit 10 deg)', final_hdg - HDG_CMD_DEG);

fprintf('All CI assertions passed.\n');
