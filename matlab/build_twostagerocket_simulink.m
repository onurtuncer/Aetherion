% build_twostagerocket_simulink.m
%
% Builds an open-loop Simulink model around a single FMU:
%
%   TwoStageRocket — Aetherion 6-DoF two-stage rocket plant (FMI 2.0,
%                    TwoStageRocketFMU.cpp), validated against
%                    NASA TM-2015-218675 Atmospheric Scenario 17.
%
% Unlike the F-16 model (plant + autopilot FMUs wired in a closed loop),
% the rocket has no separate controller FMU: staging, thrust, and Stage-2
% ignition timing are all driven internally by the DAVE-ML tables loaded
% by TwoStageRocketFMU (twostage_{inertia,prop,aero}.dml). The FMU is
% simulated open-loop for the full 200 s ascent and its trajectory is
% compared against the NASA reference CSV (Atmos_17_sim_06.csv).
%
% Environment (set up analogously to simulink.yml for the F-16 job):
%   FMIKit-Simulink/                 — FMI Kit (CATIA-Systems), repo root
%   matlab/TwoStageRocket.fmu        — plant FMU artifact from build-fmu job
%   data/Atmos_17_TwoStageRocketToOrbit/Atmos_17_sim_06.csv — NASA reference
%
% ─────────────────────────────────────────────────────────────────────────────
% TwoStageRocket FMU output port map (registration order in
% TwoStageRocketFMU.cpp; verified against the generated modelDescription.xml):
%    1  out.alt_m             12  out.v_north_m_s    23  out.aero_My_Nm
%    2  out.lat_rad           13  out.v_east_m_s      24  out.aero_Mz_Nm
%    3  out.lon_rad           14  out.v_down_m_s      25  out.mach
%    4  out.g_m_s2            15  out.a_m_s           26  out.qbar_Pa
%    5  out.yaw_rad           16  out.rho_kg_m3       27  out.vt_m_s
%    6  out.pitch_rad         17  out.P_Pa            28  out.thrust_N
%    7  out.roll_rad          18  out.T_K             29  out.mdot_kgs
%    8  out.p_rad_s           19  out.aero_Fx_N       30  out.mass_kg
%    9  out.q_rad_s           20  out.aero_Fy_N       31  out.stg1_fuel_used_kg
%   10  out.r_rad_s           21  out.aero_Fz_N       32  out.stg2_fuel_used_kg
%   11  out.altRate_m_s       22  out.aero_Mx_Nm       33  out.staged (bool)
%
% The FMU exposes no control inputs (open-loop plant, no ctrl.* ports).
% ─────────────────────────────────────────────────────────────────────────────

%% Paths
repoRoot   = pwd;
resultsDir = fullfile(repoRoot, 'matlab', 'results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

plantFmu = fullfile(repoRoot, 'matlab', 'TwoStageRocket.fmu');
refCsv   = fullfile(repoRoot, 'data', 'Atmos_17_TwoStageRocketToOrbit', 'Atmos_17_sim_06.csv');

%% FMI Kit setup
fmiKitDir = fullfile(repoRoot, 'FMIKit-Simulink');
if ~isfolder(fullfile(fmiKitDir, '+FMIKit'))
    entries = dir(repoRoot);
    error('build_twostagerocket_simulink:FMIKitNotFound', ...
        ['+FMIKit package not found at "%s".\nrepoRoot = %s\n' ...
         'Contents of repoRoot: %s'], ...
        fullfile(fmiKitDir, '+FMIKit'), repoRoot, ...
        strjoin({entries.name}, ', '));
end
addpath(fmiKitDir);
rehash toolboxcache;
FMIKit.initialize();

%% NASA reference — final-state values (Scenario 17, Atmos_17_sim_06.csv, t = 200 s)
% Converted from the reference's ft / ft-s units to SI.
FT_TO_M = 0.3048;
refTbl  = readtable(refCsv);
refRow  = refTbl(end, :);

ALT_REF_M = refRow.altitudeMsl_ft * FT_TO_M;
VT_REF_MS = sqrt(refRow.feVelocity_ft_s_X^2 + refRow.feVelocity_ft_s_Y^2 ...
                + refRow.feVelocity_ft_s_Z^2) * FT_TO_M;
STOP_TIME = refTbl.time(end);   % 200 s for Scenario 17

% Stage-2 ignition gate. The NASA reference coasts after staging and fires S2
% late enough to burn until the simulation end (matching the native
% TwoStageRocket example: endTime − post-burn coast − S2 burn duration, with
% S2 mdot ≈ 1307.34 kg/s from twostage_prop.dml and 80 000 kg of propellant).
% The FMU default (0 = ignite immediately after staging) gives a much higher
% coast apogee and will NOT match the reference final state.
STG2_MDOT_KGS      = 1307.34;
STG2_FUEL_KG       = 80000.0;
POST_BURN_COAST_S  = 7.0;
STG2_IGNITION_S    = STOP_TIME - POST_BURN_COAST_S - STG2_FUEL_KG / STG2_MDOT_KGS;

%% Create model
MDL = 'TwoStageRocket_Sim';
if bdIsLoaded(MDL), close_system(MDL, 0); end
new_system(MDL);

set_param(MDL, 'Solver','ode45', 'StopTime', num2str(STOP_TIME), ...
    'RelTol','1e-4', 'AbsTol','1e-6');

p = @(x,y,w,h) [x, y, x+w, y+h];  % position helper

% ── TwoStageRocket FMU block ─────────────────────────────────────────────────
PLANT = [MDL '/TwoStageRocket'];
add_block('FMIKit/FMU', PLANT, 'Position', p(280, 80, 160, 380));
FMIKit.loadFMU(PLANT, plantFmu);

% NASA coast-then-fire S2 sequencing + fine internal sub-stepping (the
% reference integrates at 0.01 s; without this the implicit Radau IIA stage
% takes the whole communication step at once and fails to converge).
FMIKit.setStartValue(PLANT, 'stg2.ignition_time_s', num2str(STG2_IGNITION_S));
FMIKit.setStartValue(PLANT, 'solver.max_step_s',    '0.01');

% ── To-Workspace sinks ───────────────────────────────────────────────────────
add_block('simulink/Sinks/To Workspace', [MDL '/TW_Alt'], ...
    'VariableName','alt_m',    'SaveFormat','Timeseries','Position', p(560, 80,  90, 25));
add_block('simulink/Sinks/To Workspace', [MDL '/TW_VT'], ...
    'VariableName','vt_ms',    'SaveFormat','Timeseries','Position', p(560, 120, 90, 25));
add_block('simulink/Sinks/To Workspace', [MDL '/TW_Mass'], ...
    'VariableName','mass_kg',  'SaveFormat','Timeseries','Position', p(560, 160, 90, 25));
add_block('simulink/Sinks/To Workspace', [MDL '/TW_Staged'], ...
    'VariableName','staged',   'SaveFormat','Timeseries','Position', p(560, 200, 90, 25));

% ── Wire: plant FMU outputs → workspace sinks ───────────────────────────────
% Output ports: 1=out.alt_m  27=out.vt_m_s  30=out.mass_kg  33=out.staged
add_line(MDL, 'TwoStageRocket/1',  'TW_Alt/1',    'autorouting','on');
add_line(MDL, 'TwoStageRocket/27', 'TW_VT/1',     'autorouting','on');
add_line(MDL, 'TwoStageRocket/30', 'TW_Mass/1',   'autorouting','on');
add_line(MDL, 'TwoStageRocket/33', 'TW_Staged/1', 'autorouting','on');

%% Save model
slxPath = fullfile(repoRoot, 'matlab', 'TwoStageRocket_Sim.slx');
save_system(MDL, slxPath);
fprintf('Model saved: %s\n', slxPath);

%% Run simulation
fprintf('Starting %.0f s open-loop two-stage rocket ascent simulation...\n', STOP_TIME);
simOut = sim(MDL);
fprintf('Simulation complete.\n');

%% Extract results
t        = simOut.alt_m.Time;
alt_m    = simOut.alt_m.Data;
vt_ms    = simOut.vt_ms.Data;
mass_kg  = simOut.mass_kg.Data;
staged   = simOut.staged.Data;

final_alt  = alt_m(end);
final_vt   = vt_ms(end);
final_mass = mass_kg(end);

fprintf('Final state — alt: %.1f m | vt: %.2f m/s | mass: %.1f kg | staged: %d\n', ...
        final_alt, final_vt, final_mass, staged(end));

%% Save results
save(fullfile(resultsDir, 'twostagerocket_sim_results.mat'), ...
     't', 'alt_m', 'vt_ms', 'mass_kg', 'staged', 'ALT_REF_M', 'VT_REF_MS');

%% Plot
fig = figure('Visible', 'off');

subplot(3,1,1);
plot(t, alt_m,  'b-', 'LineWidth', 1.2); hold on;
yline(ALT_REF_M, 'r--', 'LineWidth', 1);
ylabel('Altitude [m]'); legend('actual','NASA ref','Location','best'); grid on;
title('Two-Stage Rocket to Orbit — NASA TM-2015-218675 Scenario 17');

subplot(3,1,2);
plot(t, vt_ms, 'b-', 'LineWidth', 1.2); hold on;
yline(VT_REF_MS, 'r--', 'LineWidth', 1);
ylabel('TAS [m/s]'); legend('actual','NASA ref','Location','best'); grid on;

subplot(3,1,3);
plot(t, mass_kg, 'b-', 'LineWidth', 1.2);
ylabel('Mass [kg]'); xlabel('Time [s]'); grid on;

saveas(fig, fullfile(resultsDir, 'twostagerocket_ascent.png'));
close(fig);
fprintf('Results saved to %s\n', resultsDir);

%% CI assertions — final altitude/speed must track the NASA reference.
% Looser than the native (non-FMU) example's sub-1% agreement because this
% path adds FMI Co-Simulation communication-step discretization on top of
% the same underlying physics.
altErrPct = abs(final_alt - ALT_REF_M) / ALT_REF_M * 100;
vtErrPct  = abs(final_vt  - VT_REF_MS) / VT_REF_MS  * 100;

assert(altErrPct < 3.0, ...
    'Final altitude error too large: %.2f%% (limit 3%%)', altErrPct);
assert(vtErrPct < 1.0, ...
    'Final speed error too large: %.2f%% (limit 1%%)', vtErrPct);
assert(staged(end) == 1, 'Stage 1 separation did not occur by end of simulation');

fprintf('All CI assertions passed.\n');
