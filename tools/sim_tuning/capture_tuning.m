function tbl = capture_tuning(port, durationSec, outFile)
% capture_tuning  Record the FCU's [TUNE] CSV stream into a MATLAB timetable.
%
%   tbl = capture_tuning(port, durationSec, outFile)
%
%   port         e.g. "COM6" on Windows or "/dev/ttyACM0" on Linux/Mac
%   durationSec  how long to record, in seconds
%   outFile      optional: filename to save .mat (e.g. "roll_step_run1.mat")
%
%   The FCU must be flashed with -D FCU_TUNING_LOG_HZ=100 (or similar) so it
%   emits "[TUNE] t_us,ang_sp,ang_meas,rate_sp,rate_meas,p,i,d,pid_out,thr"
%   rows. Make sure no other tool (pio monitor / Python logger) is holding
%   the port open before calling this.
%
%   Columns in the returned timetable:
%     ang_sp     outer-loop angle setpoint (deg)
%     ang_meas   measured roll angle from the complementary filter (deg)
%     rate_sp    inner-loop rate setpoint (dps), output of the angle stage
%     rate_meas  measured roll rate from the gyro (dps)
%     p_term     P contribution of the rate PID (output units)
%     i_term     I contribution of the rate PID (output units)
%     d_term     D contribution of the rate PID (output units)
%     pid_out    sum P+I+D = command going to the motor mixer
%     throttle   effective throttle percent (0..100)
%
%   While capturing, give the drone a step input (push then release the
%   joystick) so the response is captured at a known moment. 10-30 s is
%   typical; longer captures fit the plant better.

    if nargin < 2
        error("Usage: capture_tuning(port, durationSec, [outFile])");
    end

    fprintf("Opening %s @ 115200 ...\n", port);
    sp = serialport(port, 115200, "Timeout", 0.5);
    configureTerminator(sp, "LF");
    flush(sp);
    cleanup = onCleanup(@() clear('sp'));

    fprintf("Capturing for %.1f s. Send a step input from the remote NOW.\n", durationSec);
    rows = strings(0,1);
    t0 = tic;
    while toc(t0) < durationSec
        if sp.NumBytesAvailable > 0
            line = readline(sp);
            if startsWith(line, "[TUNE] ")
                rows(end+1,1) = line; %#ok<AGROW>
            end
        else
            pause(0.001);
        end
    end
    fprintf("Captured %d sample rows.\n", numel(rows));
    if numel(rows) < 10
        error("Too few samples — is FCU_TUNING_LOG_HZ set and the firmware flashed?");
    end

    % Parse "[TUNE] " prefix off, sscanf the comma-separated floats.
    n = numel(rows);
    data = nan(n, 10);
    for i = 1:n
        s = char(rows(i));
        s = s(8:end);            % strip "[TUNE] "
        v = sscanf(s, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f");
        if numel(v) == 10
            data(i, :) = v.';
        end
    end
    data = data(all(~isnan(data), 2), :);

    % Convert t_us to seconds-since-start, build timetable.
    t_s = (data(:,1) - data(1,1)) * 1e-6;
    vars = {'ang_sp','ang_meas','rate_sp','rate_meas', ...
            'p_term','i_term','d_term','pid_out','throttle'};
    tbl = timetable(seconds(t_s), data(:,2),data(:,3),data(:,4),data(:,5), ...
                                  data(:,6),data(:,7),data(:,8),data(:,9), ...
                                  data(:,10), ...
                    'VariableNames', vars);
    tbl.Properties.Description = sprintf("FCU [TUNE] log captured %s", datetime("now"));

    if nargin >= 3 && ~isempty(outFile)
        save(outFile, "tbl");
        fprintf("Saved to %s\n", outFile);
    end
end
