function figs = flight_plot(logFile)
%FLIGHT_PLOT  Plot a TVC flight computer log.
%   FLIGHT_PLOT opens a file picker.
%   FLIGHT_PLOT(FILE) plots FILE.
%   FIGS = FLIGHT_PLOT(...) also returns the three figure handles.
%
%   Figures:
%     1. Flight path - estimated 3D trajectory, coloured by flight state,
%        with liftoff, burnout, apogee and parachute events marked.
%     2. Flight data - altitude, vertical velocity, acceleration, pressure
%        and temperature against time from liftoff.
%     3. Control - what each PID gain acted on: the tilt input to each
%        axis, the P, I and D terms rebuilt from the logged tilt with the
%        gains in include/pid.h, the logged PID output, and the servo
%        commands.
%
%   Reads the FLIGHT_XXX.CSV the flight computer writes to the SD card
%   (comma- or tab-separated, so a copy re-saved from Excel works too),
%   and the recordings tools/flight_live.py makes from telemetry.
%
%   About the 3D path: the board has no GPS, so only altitude is measured.
%   Horizontal position is estimated by assuming the rocket flies along
%   its own axis during powered flight and coast (zero angle of attack):
%   horizontal speed = vertical speed x tan(tilt), in the direction the
%   nose leans. There's no horizontal information after apogee, so the
%   descent is drawn straight down. "North" is magnetic north if the
%   magnetometer set the heading at boot, otherwise the direction the
%   board faced at power-up.

if nargin < 1 || isempty(logFile)
    [name, folder] = uigetfile({'*.csv;*.CSV;*.txt', 'Flight logs (*.csv, *.txt)'}, ...
                               'Pick a flight log');
    if isequal(name, 0)
        figs = gobjects(0);
        return
    end
    logFile = fullfile(folder, name);
end

T   = read_log(logFile);
cfg = read_firmware_config();
d   = derive(T, cfg);

[~, base, ext] = fileparts(logFile);
label = [base ext];

print_summary(d, cfg, label);
figs = [plot_path(d, label), plot_data(d, label), plot_control(d, cfg, label)];
if nargout == 0
    clear figs
end
end

% =====================================================================
%  Loading
% =====================================================================

function T = read_log(file)
if ~isfile(file)
    error('flight_plot:noFile', 'Log not found: %s', file);
end
opts = detectImportOptions(file, 'FileType', 'text');
opts = setvartype(opts, 'double');       % "nan" cells import as NaN
T = readtable(file, opts);

need = {'timestamp_ms', 'roll', 'pitch', 'yaw', 'q0', 'q1', 'q2', 'q3', ...
        'gx', 'gy', 'gz', 'ax', 'ay', 'az', 'temperature_c', 'pressure_hpa', ...
        'altitude_m', 'velocity_ms', 'servo_pitch_us', 'servo_yaw_us', ...
        'pid_pitch_out', 'pid_yaw_out', 'flight_state'};
missing = setdiff(need, T.Properties.VariableNames);
if ~isempty(missing)
    error('flight_plot:columns', '%s is missing columns: %s', file, strjoin(missing, ', '));
end

T = T(~isnan(T.timestamp_ms), :);        % blank or garbled lines
if height(T) < 2
    error('flight_plot:empty', '%s has fewer than 2 records', file);
end
% Arduino prints "inf" / "ovf" for out-of-range floats - treat as missing
for v = T.Properties.VariableNames
    x = T.(v{1});
    x(isinf(x)) = NaN;
    T.(v{1}) = x;
end
end

function cfg = read_firmware_config()
% Gains and limits as the firmware defines them, so the P/I/D rebuild uses
% the same numbers. Falls back to these values if the script isn't in the
% repo's tools/ folder.
cfg = struct('kp', 0.34, 'ki', 0.215, 'kd', 0.05, 'ilim', 8.5, ...
             'lim_p', 7, 'lim_y', 10, 'src', 'built-in defaults');
inc = fullfile(fileparts(mfilename('fullpath')), '..', 'include');
pidFile   = fullfile(inc, 'pid.h');
servoFile = fullfile(inc, 'servo_driver.h');
if isfile(pidFile)
    txt = fileread(pidFile);
    cfg.kp   = define_value(txt, 'PID_DEFAULT_KP', cfg.kp);
    cfg.ki   = define_value(txt, 'PID_DEFAULT_KI', cfg.ki);
    cfg.kd   = define_value(txt, 'PID_DEFAULT_KD', cfg.kd);
    cfg.ilim = define_value(txt, 'PID_INTEGRAL_LIMIT', cfg.ilim);
    cfg.src  = 'include/pid.h';
end
if isfile(servoFile)
    txt = fileread(servoFile);
    cfg.lim_p = define_value(txt, 'SERVO_PITCH_MAX_ANGLE_DEG', cfg.lim_p);
    cfg.lim_y = define_value(txt, 'SERVO_YAW_MAX_ANGLE_DEG', cfg.lim_y);
end
end

function v = define_value(txt, name, fallback)
tok = regexp(txt, ['#define\s+' name '\s+([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)'], 'tokens', 'once');
if isempty(tok)
    v = fallback;
else
    v = str2double(tok{1});
end
end

% =====================================================================
%  Derived quantities
% =====================================================================

function d = derive(T, cfg)
n = height(T);
d.n = n;
st = T.flight_state;
st(isnan(st)) = 0;
d.state = round(st);

% Time from liftoff confirmation (first POWERED record), or from the first
% record if the log has no flight in it.
iLaunch = find(d.state == 2, 1);
d.has_launch = ~isempty(iLaunch);
if d.has_launch
    t0 = T.timestamp_ms(iLaunch);
    d.tlabel = 'Time from liftoff [s]';
else
    t0 = T.timestamp_ms(1);
    d.tlabel = 'Time [s]';
end
d.t = (T.timestamp_ms - t0) / 1000;

dt = [0; diff(d.t)];
ok = dt > 0 & dt < 1;
if any(ok), nominal = median(dt(ok)); else, nominal = 0.008; end
dt(~ok) = nominal;                       % first record, gaps
d.dt = dt;

d.alt   = T.altitude_m;
d.vel   = T.velocity_ms;
d.press = T.pressure_hpa;
d.temp  = T.temperature_c;
d.ax = T.ax;  d.ay = T.ay;  d.az = T.az;
d.amag = sqrt(d.ax.^2 + d.ay.^2 + d.az.^2);

% Attitude. The filter frame has X/Y horizontal and Z along the airframe
% toward the nozzle; R(q) turns it into NED (see mahrs_integration.cpp).
q  = [T.q0 T.q1 T.q2 T.q3];
qn = sqrt(sum(q.^2, 2));
bad = ~(qn > 0.5);                       % zeros / NaN -> upright
q(bad, :) = repmat([1 0 0 0], nnz(bad), 1);
qn(bad) = 1;
q = q ./ qn;
q0 = q(:, 1); q1 = q(:, 2); q2 = q(:, 3); q3 = q(:, 4);

% Nose direction in NED = -R(:,3)
nN = -2 * (q1 .* q3 + q0 .* q2);
nE = -2 * (q2 .* q3 - q0 .* q1);
nU =  1 - 2 * (q1.^2 + q2.^2);           % = cos(tilt)
d.tilt = acosd(max(-1, min(1, nU)));

% Upward specific force in the earth frame [g] - what the FSM's launch and
% burnout checks use: 1 g at rest whichever way the board lies.
fx = T.az;  fy = -T.ay;  fz = T.ax;
d.a_up = -(2 * (q1 .* q3 - q0 .* q2) .* fx + 2 * (q2 .* q3 + q0 .* q1) .* fy + nU .* fz);

% Horizontal velocity during powered flight and coast, from the nose
% direction (zero angle of attack). Tilts past 75 deg are skipped - tan()
% blows up there and the assumption doesn't hold anyway.
asc = (d.state == 2 | d.state == 3) & nU > cosd(75);
vup = max(d.vel, 0);
vup(isnan(vup)) = 0;
vN = zeros(n, 1);
vE = zeros(n, 1);
vN(asc) = vup(asc) .* nN(asc) ./ nU(asc);
vE(asc) = vup(asc) .* nE(asc) ./ nU(asc);
d.N = cumsum(vN .* dt);
d.E = cumsum(vE .* dt);
d.U = d.alt;

% Control. The CSV's "pitch" column is tip_a (the pitch PID's input) and
% "roll" is tip_b (the yaw PID's input) - see main_control_loop.cpp.
d.tip_a   = T.pitch;
d.tip_b   = T.roll;
d.pid_p   = T.pid_pitch_out;
d.pid_y   = T.pid_yaw_out;
d.servo_p = T.servo_pitch_us;
d.servo_y = T.servo_yaw_us;
nonzero = @(x) ~isnan(x) & x ~= 0;
d.tvc = d.state == 2 | nonzero(d.pid_p) | nonzero(d.pid_y);
d.pitch = pid_replay(d.tip_a, d.tvc, dt, cfg.kp, cfg.ki, cfg.kd, cfg.lim_p, min(cfg.ilim, cfg.lim_p));
d.yaw   = pid_replay(d.tip_b, d.tvc, dt, cfg.kp, cfg.ki, cfg.kd, cfg.lim_y, min(cfg.ilim, cfg.lim_y));

d.events = find_events(d);
end

function r = pid_replay(meas, active, dt, kp, ki, kd, lim, ilim)
% Re-runs pid_update() from src/pid.cpp on the logged tilt. The firmware
% resets both controllers when TVC switches on and outputs exactly 0 while
% it's off, so "active" is: POWERED, or a non-zero logged output.
n = numel(meas);
r.P = nan(n, 1);  r.I = r.P;  r.D = r.P;  r.out = r.P;
integ = 0;  prevErr = 0;  first = true;  was = false;
for k = 1:n
    if ~active(k) || isnan(meas(k))
        was = false;
        continue
    end
    if ~was                               % rising edge: pid_reset()
        integ = 0;  prevErr = 0;  first = true;
    end
    was = true;
    e = -meas(k);                         % setpoint is 0
    r.P(k) = kp * e;
    integ  = min(max(integ + ki * e * dt(k), -ilim), ilim);
    r.I(k) = integ;
    if first
        r.D(k) = 0;
    else
        r.D(k) = kd * (e - prevErr) / dt(k);
    end
    first = false;
    prevErr = e;
    r.out(k) = min(max(r.P(k) + r.I(k) + r.D(k), -lim), lim);
end
end

function ev = find_events(d)
defs = {2, 'Liftoff'; 3, 'Burnout'; 4, 'Apogee detected'; 6, 'Main chute'; 8, 'ABORT'};
ev = struct('name', {}, 'i', {});
for k = 1:size(defs, 1)
    i = find(d.state == defs{k, 1}, 1);
    if ~isempty(i)
        ev(end + 1) = struct('name', defs{k, 2}, 'i', i); %#ok<AGROW>
    end
end
if d.has_launch
    [~, iMax] = max(d.alt);
    iApo = find(d.state == 4, 1);
    if isempty(iApo) || abs(d.t(iMax) - d.t(iApo)) > 0.5     % else the labels would collide
        ev(end + 1) = struct('name', 'Max altitude', 'i', iMax);
    end
    if any(d.state(end) == [5 6 7])
        ev(end + 1) = struct('name', 'Landing (log ends)', 'i', d.n);
    end
end
[~, order] = sort([ev.i]);
ev = ev(order);
end

% =====================================================================
%  Summary
% =====================================================================

function print_summary(d, cfg, label)
fprintf('\n=== %s ===\n', label);
fprintf('  %d records over %.1f s (%.0f Hz)\n', d.n, d.t(end) - d.t(1), 1 / median(d.dt));
if ~d.has_launch
    fprintf('  No liftoff in this log (flight_state never reached POWERED) - times are from the first record.\n');
end
[v, i] = max(d.alt);   fprintf('  Max altitude        %8.1f m     at T%+.2f s\n', v, d.t(i));
[v, i] = max(d.vel);   fprintf('  Max vertical speed  %8.1f m/s   at T%+.2f s\n', v, d.t(i));
[v, i] = max(d.amag);  fprintf('  Max |acceleration|  %8.1f g     at T%+.2f s\n', v, d.t(i));
iP = find(d.state == 2, 1);
iC = find(d.state == 3, 1);
if ~isempty(iP) && ~isempty(iC)
    fprintf('  Burn time           %8.2f s\n', d.t(iC) - d.t(iP));
end
asc = d.state == 2 | d.state == 3;
if any(asc)
    fprintf('  Max tilt, ascent    %8.1f deg\n', max(d.tilt(asc)));
    [~, ia] = max(d.alt);
    fprintf('  Est. drift at apogee %7.1f m     (no GPS - see help flight_plot)\n', hypot(d.E(ia), d.N(ia)));
end
m = d.state == 6;
if nnz(m) > 10
    fprintf('  Descent rate, main  %8.1f m/s\n', -median(d.vel(m), 'omitnan'));
end
fprintf('  PID gains (%s): Kp %g  Ki %g  Kd %g\n', cfg.src, cfg.kp, cfg.ki, cfg.kd);
if any(d.tvc)
    ep = max(abs(d.pitch.out(d.tvc) - d.pid_p(d.tvc)), [], 'omitnan');
    ey = max(abs(d.yaw.out(d.tvc)   - d.pid_y(d.tvc)), [], 'omitnan');
    fprintf('  PID rebuild vs logged output: max diff %.3f deg pitch, %.3f deg yaw\n', ep, ey);
    if max(ep, ey) > 0.5
        fprintf('  -> More than 0.5 deg apart: the gains in %s may not be the ones that flew.\n', cfg.src);
    end
end
fprintf('\n');
end

% =====================================================================
%  Figures
% =====================================================================

function [names, colors] = state_style()
names  = {'IDLE', 'ARMED', 'POWERED', 'COAST', 'APOGEE', 'DESCENT', 'MAIN', 'LANDED', 'ABORT'};
colors = [0.60 0.60 0.60     % IDLE
          0.40 0.40 0.40     % ARMED
          0.85 0.33 0.10     % POWERED
          0.00 0.45 0.74     % COAST
          0.93 0.69 0.13     % APOGEE
          0.49 0.18 0.56     % DESCENT
          0.47 0.67 0.19     % MAIN
          0.25 0.25 0.25     % LANDED
          0.90 0.00 0.00];   % ABORT
end

function fig = plot_path(d, label)
[names, colors] = state_style();
fig = figure('Name', ['Flight path - ' label], 'NumberTitle', 'off', 'Color', 'w');
ax = axes(fig);
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

plot3(ax, d.E, d.N, zeros(d.n, 1), ':', 'Color', [0.7 0.7 0.7], 'DisplayName', 'Ground track');
for s = unique(d.state)'
    m = d.state == s;
    m = m | [false; m(1:end - 1)];       % include the next point so runs join up
    x = d.E;  y = d.N;  z = d.U;
    x(~m) = NaN;  y(~m) = NaN;  z(~m) = NaN;
    if s >= 0 && s < numel(names)
        nm = names{s + 1};  c = colors(s + 1, :);
    else
        nm = sprintf('state %d', s);  c = [0.5 0.5 0.5];
    end
    plot3(ax, x, y, z, '-', 'Color', c, 'LineWidth', 2.2, 'DisplayName', nm);
end
plot3(ax, 0, 0, 0, 'k^', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Pad');
for k = 1:numel(d.events)
    i = d.events(k).i;
    plot3(ax, d.E(i), d.N(i), d.U(i), 'ko', 'MarkerFaceColor', [1 0.9 0.2], ...
          'MarkerSize', 7, 'HandleVisibility', 'off');
    text(ax, d.E(i), d.N(i), d.U(i), ['   ' d.events(k).name], 'FontSize', 9);
end

% Same horizontal scale on both axes, never narrower than 15% of apogee,
% so a small drift isn't stretched into a dramatic curve.
hs = max([abs(d.E); abs(d.N); 0.15 * max(d.U); 5]);
xlim(ax, [-hs hs]);
ylim(ax, [-hs hs]);
zlim(ax, [min(0, min(d.U)), max(1.05 * max(d.U), 1)]);
xlabel(ax, 'East [m]');  ylabel(ax, 'North [m]');  zlabel(ax, 'Altitude AGL [m]');
view(ax, 40, 22);
legend(ax, 'Location', 'northeastoutside');
title(ax, {['Estimated flight path - ' label], ...
           'Altitude measured; horizontal estimated from tilt x vertical speed (no GPS)'}, ...
      'Interpreter', 'none');
end

function fig = plot_data(d, label)
fig = figure('Name', ['Flight data - ' label], 'NumberTitle', 'off', 'Color', 'w');
tl = tiledlayout(fig, 5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, ['Flight data - ' label], 'Interpreter', 'none');
axs = gobjects(5, 1);

axs(1) = nexttile(tl);
plot(axs(1), d.t, d.alt, 'LineWidth', 1.3);
ylabel(axs(1), 'Altitude [m]');

axs(2) = nexttile(tl);
plot(axs(2), d.t, d.vel, 'LineWidth', 1.3);
yline(axs(2), 0, ':', 'HandleVisibility', 'off');
ylabel(axs(2), 'Vert. speed [m/s]');

axs(3) = nexttile(tl);
plot(axs(3), d.t, [d.ax d.ay d.az], 'LineWidth', 1);
hold(axs(3), 'on');
plot(axs(3), d.t, d.amag, 'k', 'LineWidth', 1.3);
plot(axs(3), d.t, d.a_up, '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.1);
ylabel(axs(3), 'Accel [g]');
lgd = legend(axs(3), {'a_x (body, + toward nozzle)', 'a_y', 'a_z', '|a|', 'up, earth frame'}, ...
             'FontSize', 8);
lgd.Layout.Tile = 'east';               % beside the layout, so it never covers data

axs(4) = nexttile(tl);
m = ~isnan(d.press);
plot(axs(4), d.t(m), d.press(m), 'LineWidth', 1.3);
ylabel(axs(4), 'Pressure [hPa]');

axs(5) = nexttile(tl);
m = ~isnan(d.temp);
plot(axs(5), d.t(m), d.temp(m), 'LineWidth', 1.3);
ylabel(axs(5), 'Temp [°C]');
xlabel(axs(5), d.tlabel);

for a = axs'
    grid(a, 'on');
end
linkaxes(axs, 'x');
xlim(axs(1), [d.t(1) d.t(end)]);
add_event_lines(axs, d);
end

function fig = plot_control(d, cfg, label)
fig = figure('Name', ['Control - ' label], 'NumberTitle', 'off', 'Color', 'w');
tl = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, {sprintf('PID gains: Kp = %g    Ki = %g    Kd = %g    (from %s)', cfg.kp, cfg.ki, cfg.kd, cfg.src), ...
           'P, I and D are rebuilt from the logged tilt with these gains; the logged output is what the flight computer commanded'}, ...
      'Interpreter', 'none', 'FontSize', 10);

axs = gobjects(5, 1);
axs(1) = nexttile(tl);
pid_axes(axs(1), d, d.tip_a, d.pitch, d.pid_p, cfg.lim_p, 'Pitch PID - input tip_a (CSV "pitch")');
axs(2) = nexttile(tl);
pid_axes(axs(2), d, d.tip_b, d.yaw, d.pid_y, cfg.lim_y, 'Yaw PID - input tip_b (CSV "roll")');
lgd = legend(axs(1), 'Orientation', 'horizontal', 'FontSize', 8);   % same entries on both axes
lgd.Layout.Tile = 'north';

axs(3) = nexttile(tl);
plot(axs(3), d.t, d.servo_p, 'LineWidth', 1.2);
ylabel(axs(3), 'Pitch servo [µs]');
axs(4) = nexttile(tl);
plot(axs(4), d.t, d.servo_y, 'LineWidth', 1.2);
ylabel(axs(4), 'Yaw servo [µs]');

axs(5) = nexttile(tl, [1 2]);
hold(axs(5), 'on');
yl = [0, max(5, 1.1 * max(d.tilt))];
shade_runs(axs(5), d.t, d.tvc, yl, [0.85 0.33 0.10]);
plot(axs(5), d.t, d.tilt, 'k', 'LineWidth', 1.3, 'DisplayName', 'Total tilt from vertical');
ylim(axs(5), yl);
ylabel(axs(5), 'Tilt [deg]');
xlabel(axs(5), d.tlabel);
legend(axs(5), 'Location', 'northwest', 'FontSize', 8);

for a = axs'
    grid(a, 'on');
end
linkaxes(axs, 'x');
% Zoom to the steering window when there is one
if any(d.tvc)
    xlim(axs(1), [d.t(find(d.tvc, 1)) - 1, d.t(find(d.tvc, 1, 'last')) + 2]);
else
    xlim(axs(1), [d.t(1) d.t(end)]);
end
add_event_lines(axs, d);
end

function pid_axes(ax, d, input, r, logged, lim, ttl)
hold(ax, 'on');
plot(ax, d.t, input,  'Color', 'k',                 'LineWidth', 1.4, 'DisplayName', 'Input: tilt (error = -tilt)');
plot(ax, d.t, r.P,    'Color', [0.00 0.45 0.74],    'LineWidth', 1.0, 'DisplayName', 'P = Kp·e');
plot(ax, d.t, r.I,    'Color', [0.93 0.55 0.10],    'LineWidth', 1.2, 'DisplayName', 'I = Ki·∫e dt');
plot(ax, d.t, r.D,    'Color', [0.49 0.18 0.56 0.6], 'LineWidth', 0.8, 'DisplayName', 'D = Kd·de/dt');
plot(ax, d.t, logged, 'Color', [0.90 0.00 0.00],    'LineWidth', 1.6, 'DisplayName', 'Logged output (nozzle deg)');
yline(ax, [-lim lim], ':', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
title(ax, ttl, 'Interpreter', 'none', 'FontWeight', 'normal');
ylabel(ax, 'deg');
end

function shade_runs(ax, t, mask, yl, color)
% One translucent band per run of true values in mask.
edges = diff([false; mask(:); false]);
starts = find(edges == 1);
stops  = find(edges == -1) - 1;
for k = 1:numel(starts)
    x0 = t(starts(k));  x1 = t(stops(k));
    h = patch(ax, [x0 x1 x1 x0], [yl(1) yl(1) yl(2) yl(2)], color, ...
              'FaceAlpha', 0.15, 'EdgeColor', 'none', 'DisplayName', 'TVC steering');
    if k > 1
        h.HandleVisibility = 'off';
    end
end
end

function add_event_lines(axs, d)
for k = 1:numel(d.events)
    x = d.t(d.events(k).i);
    for j = 1:numel(axs)
        if j == 1
            xline(axs(j), x, ':', d.events(k).name, 'Color', [0.35 0.35 0.35], ...
                  'FontSize', 8, 'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
        else
            xline(axs(j), x, ':', 'Color', [0.35 0.35 0.35], 'HandleVisibility', 'off');
        end
    end
end
end
