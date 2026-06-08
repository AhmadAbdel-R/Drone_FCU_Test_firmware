function plant = study_pid(tblOrFile)
% study_pid  Plot the captured FCU PID data, fit a plant, build a Simulink model.
%
%   plant = study_pid(tbl)       % pass the timetable from capture_tuning
%   plant = study_pid("file.mat") % or load it from disk
%
%   Side effects:
%     - opens a 3-panel figure (angle, rate, PID terms over time)
%     - prints fit quality, suggested gains
%     - builds and opens a Simulink model "fcu_roll_sim.slx" with the
%       cascaded angle/rate loop and the identified plant
%     - writes 'rate_plant', 'angle_plant', 'setpoint_ts' into the base
%       workspace so the Simulink model can use them via From Workspace
%
%   Returns the identified rate-loop transfer function.

    if isstring(tblOrFile) || ischar(tblOrFile)
        load(tblOrFile, "tbl");
    else
        tbl = tblOrFile;
    end
    assert(istimetable(tbl), "Expected a timetable from capture_tuning");

    t = seconds(tbl.Time);
    Ts = mean(diff(t));
    fprintf("\nLoaded %d samples, Ts = %.2f ms (%.0f Hz effective)\n", ...
            height(tbl), Ts*1e3, 1/Ts);

    % ---- Three-panel time series -----------------------------------------
    figure('Name','Captured FCU tuning data','NumberTitle','off',...
           'Position',[80 80 1200 880]);
    tl = tiledlayout(3,1,'TileSpacing','compact');
    title(tl, sprintf("FCU roll tuning capture (%.1f s, %.0f Hz log)", t(end), 1/Ts));

    nexttile;
    plot(t, tbl.ang_sp, 'b', 'LineWidth',1.2); hold on;
    plot(t, tbl.ang_meas, 'r', 'LineWidth',1.0);
    legend('Setpoint','Measured','Location','best');
    grid on; ylabel('Roll angle (deg)');
    title('Outer (angle) loop');

    nexttile;
    plot(t, tbl.rate_sp, 'b', 'LineWidth',1.2); hold on;
    plot(t, tbl.rate_meas, 'r', 'LineWidth',1.0);
    legend('Setpoint','Measured','Location','best');
    grid on; ylabel('Roll rate (dps)');
    title('Inner (rate) loop');

    nexttile;
    plot(t, tbl.p_term, t, tbl.i_term, t, tbl.d_term, t, tbl.pid_out, 'LineWidth',1.0);
    legend('P','I','D','Sum','Location','best');
    grid on; ylabel('PID terms');
    xlabel('time (s)');
    title('Rate PID internal terms');

    % ---- System identification: rate-loop plant --------------------------
    % Model the SISO map from pid_out (motor-mixer command) to measured rate.
    % 2 poles, 0 zeros is a good starting guess for a quad rate loop
    % (motor first-order pole + frame inertia/damping pole). tfest tries
    % continuous-time by default.
    u = tbl.pid_out;
    y = tbl.rate_meas;
    % Drop the first ~200 ms so the I-term steady-state doesn't pollute the fit.
    keepFrom = find(t > 0.2, 1);
    if isempty(keepFrom), keepFrom = 1; end
    idData = iddata(y(keepFrom:end), u(keepFrom:end), Ts);
    idData.InputName  = {'pid_out'};
    idData.OutputName = {'rate_dps'};

    fprintf("\nFitting 2-pole rate plant via tfest ...\n");
    plant = tfest(idData, 2, 0);
    fitPct = plant.Report.Fit.FitPercent;
    fprintf("Plant identified. Fit: %.1f%% (higher = better)\n", fitPct);
    if fitPct < 50
        warning("Low fit — try a longer capture with a clean step input.");
    end
    disp(plant);

    % Angle plant is just an integrator by definition (rate -> angle), so
    % we don't need to identify it; Simulink will use a discrete-time 1/s.

    % ---- Suggest gains via Simulink pidtune ------------------------------
    fprintf("\nAuto-tuning a candidate rate PID (pidtune, target 0.6 phase margin) ...\n");
    try
        opts = pidtuneOptions('PhaseMargin', 60, 'DesignFocus', 'balanced');
        Cauto = pidtune(plant, 'PID', opts);
        fprintf("  Suggested rate PID:  P = %.4f   I = %.4f   D = %.4f\n", ...
                Cauto.Kp, Cauto.Ki, Cauto.Kd);
        fprintf("  (these are in milli-units they need * 1000 for the remote display)\n");
    catch ME
        warning(ME.identifier, "pidtune failed: %s", ME.message);
        Cauto = pid(0.04, 0, 0);
    end

    % ---- Push artefacts to base workspace and build the model ------------
    setpoint_ts = timeseries(tbl.ang_sp, t);
    setpoint_ts.Name = 'roll_angle_setpoint_deg';
    assignin('base', 'rate_plant',  plant);
    assignin('base', 'setpoint_ts', setpoint_ts);
    assignin('base', 'measured_angle_ts', timeseries(tbl.ang_meas, t));
    assignin('base', 'measured_rate_ts',  timeseries(tbl.rate_meas, t));
    assignin('base', 'pidtune_kp', Cauto.Kp);
    assignin('base', 'pidtune_ki', Cauto.Ki);
    assignin('base', 'pidtune_kd', Cauto.Kd);

    fprintf("\nBuilding Simulink model 'fcu_roll_sim'...\n");
    build_pid_sim(plant, t(end), Cauto.Kp, Cauto.Ki, Cauto.Kd);

    fprintf("\nDone. Workflow:\n");
    fprintf("  1. The Simulink model is now open. Hit Run to simulate.\n");
    fprintf("  2. Double-click the Rate PID block, click 'Tune' to launch PID Tuner.\n");
    fprintf("  3. Drag the sliders, observe simulated response.\n");
    fprintf("  4. Best gains found -> apply them via the remote's settings list.\n");
end
