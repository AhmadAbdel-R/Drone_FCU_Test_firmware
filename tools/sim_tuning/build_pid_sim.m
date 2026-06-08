function build_pid_sim(ratePlant, simStopTime, kp, ki, kd)
% build_pid_sim  Programmatically construct the cascaded roll-axis Simulink
% model used to study FCU PID tuning.
%
%   build_pid_sim(ratePlant, simStopTime, kp, ki, kd)
%
%   ratePlant     a `tf` object (output of tfest), maps pid_out -> rate_dps
%   simStopTime   seconds; usually the length of your capture
%   kp, ki, kd    initial rate-PID gains (typically from pidtune)
%
%   The constructed model has the structure:
%
%      Setpoint ─►(+)── × angleGain ──► Rate Sat ──►(+)── PID ──► Plant ─┬──► Rate Scope
%                  ▲                                  ▲                  │
%                  │  (-)                             │  (-)             ▼
%                  │                                  └──────────────────┘
%                  │                                                     │
%                  │                                                     ▼
%                  └────────────────────────────────────── ◄── 1/s (Integrator)──► Angle Scope
%
%   The Setpoint comes from base-workspace `setpoint_ts` (a timeseries the
%   capture script populated with the angle setpoint history). That way the
%   simulation re-plays whatever stick input you flew during capture, and
%   you can compare simulated angle to the measured angle saved as
%   `measured_angle_ts`.
%
%   Tweakable parameters:
%     - The Rate PID block exposes a Tune button → MATLAB PID Tuner GUI.
%     - The Angle Gain block (default 5.0) is the outer-loop P. Match it to
%       what your firmware uses if you want quantitative agreement.

    name = "fcu_roll_sim";
    if bdIsLoaded(name); bdclose(name, 0); end
    new_system(name);
    open_system(name);

    add_block('simulink/Sources/From Workspace', sprintf('%s/Angle Setpoint', name), ...
              'VariableName', 'setpoint_ts', ...
              'Position', [40 100 130 140]);

    add_block('simulink/Math Operations/Sum', sprintf('%s/Angle Err', name), ...
              'Inputs', '+-', 'IconShape','round','Position', [170 105 200 135]);

    add_block('simulink/Math Operations/Gain', sprintf('%s/Angle P', name), ...
              'Gain', '5.0', 'Position', [230 100 290 140]);

    add_block('simulink/Discontinuities/Saturation', sprintf('%s/Rate Sat', name), ...
              'UpperLimit', '150', 'LowerLimit', '-150', ...
              'Position', [320 100 380 140]);

    add_block('simulink/Math Operations/Sum', sprintf('%s/Rate Err', name), ...
              'Inputs', '+-', 'IconShape','round','Position', [410 105 440 135]);

    add_block('simulink/Continuous/PID Controller', sprintf('%s/Rate PID', name), ...
              'P', num2str(kp,'%.6f'), 'I', num2str(ki,'%.6f'), 'D', num2str(kd,'%.6f'), ...
              'Position', [470 90 540 150]);

    numStr = mat2str(ratePlant.Numerator);
    denStr = mat2str(ratePlant.Denominator);
    add_block('simulink/Continuous/Transfer Fcn', sprintf('%s/Rate Plant', name), ...
              'Numerator', numStr, 'Denominator', denStr, ...
              'Position', [580 90 680 150]);

    add_block('simulink/Continuous/Integrator', sprintf('%s/Rate to Angle', name), ...
              'Position', [720 100 760 140]);

    add_block('simulink/Sinks/Scope', sprintf('%s/Angle Scope', name), ...
              'Position', [800 100 840 140]);
    add_block('simulink/Sinks/Scope', sprintf('%s/Rate Scope', name), ...
              'Position', [720 30 760 70]);

    % Wires
    add_line(name, 'Angle Setpoint/1',  'Angle Err/1', 'autorouting','on');
    add_line(name, 'Angle Err/1',       'Angle P/1',   'autorouting','on');
    add_line(name, 'Angle P/1',         'Rate Sat/1',  'autorouting','on');
    add_line(name, 'Rate Sat/1',        'Rate Err/1',  'autorouting','on');
    add_line(name, 'Rate Err/1',        'Rate PID/1',  'autorouting','on');
    add_line(name, 'Rate PID/1',        'Rate Plant/1','autorouting','on');
    add_line(name, 'Rate Plant/1',      'Rate to Angle/1','autorouting','on');
    add_line(name, 'Rate to Angle/1',   'Angle Scope/1','autorouting','on');
    add_line(name, 'Rate Plant/1',      'Rate Scope/1','autorouting','on');

    % Feedback paths
    add_line(name, 'Rate to Angle/1',   'Angle Err/2','autorouting','on');
    add_line(name, 'Rate Plant/1',      'Rate Err/2','autorouting','on');

    % Sim configuration
    set_param(name, 'StopTime',     num2str(simStopTime));
    set_param(name, 'Solver',       'ode45');
    set_param(name, 'MaxStep',      '0.001');
    set_param(name, 'SaveOutput',   'off');

    % Save next to the script.
    here = fileparts(mfilename('fullpath'));
    save_system(name, fullfile(here, char(name)));
    open_system(name);

    fprintf("Simulink model saved to %s\n", fullfile(here, name + ".slx"));
end
