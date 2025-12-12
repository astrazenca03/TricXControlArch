function [t_log, x_log, F_log] = runTriRotorUDP(Tsim)
% runTriRotorUDP  Closed-loop LQR control of TriRotor via JSBSim.
%
%   [t_log, x_log, F_log] = runTriRotorUDP(Tsim)
%       Tsim [s] = desired simulation duration (JSBSim sim time).
%
% JSBSim side (TriRotor.xml):
%   OUTPUT (UDP 5501, 50 Hz):
%       simulation/sim-time-sec
%       velocities/p-rad_sec
%       velocities/q-rad_sec
%       velocities/r-rad_sec
%       attitude/phi-rad
%       attitude/theta-rad
%       attitude/psi-rad
%       fcs/rotor1-thrust
%       fcs/rotor2-thrust
%       fcs/rotor3-thrust
%
%   INPUT (TCP 5502):
%       fcs/rotor1-thrust
%       fcs/rotor2-thrust
%       fcs/rotor3-thrust
%
% MATLAB side:
%   - Reads UDP from 5501 as ASCII line, whitespace or comma separated.
%   - Computes tau = -K_rot * (x - x_ref), with gain ramp.
%   - Maps tau -> dF via B_inv, optionally enforces sum(dF)=0.
%   - Sends updated F1..F3 via TCP "set" commands.

    %% === Load controller design ===
    S = load("trirotor_gains.mat", "K_rot", "B_inv", "F0_hover");
    K_rot    = S.K_rot;       % 3x6
    B_inv    = S.B_inv;       % 3x3
    F0_hover = S.F0_hover(:); % 3x1

    if ~isequal(size(K_rot), [3 6])
        error("K_rot in trirotor_gains.mat must be 3x6.");
    end
    if ~isequal(size(B_inv), [3 3])
        error("B_inv in trirotor_gains.mat must be 3x3.");
    end
    if ~isequal(numel(F0_hover), 3)
        error("F0_hover in trirotor_gains.mat must be 3 elements.");
    end

    %% === Tuning knobs ===
    gain_scale_final = 0.25;   % final fraction of LQR gain (e.g. 0.25 = 25%)
    ramp_time        = 2.0;    % [s] time to ramp from 0 -> gain_scale_final
    zero_sum_dF      = true;   % enforce sum(dF) = 0 to avoid changing total thrust
    invert_LM        = true;   % flip signs of roll/pitch torques if needed

    % Torque saturation [N*m]
    tau_max_roll_pitch = 0.20;
    tau_max_yaw        = 0.05;

    % Rotor thrust limits [N]
    Fmin = 0.0;
    Fmax = 20.0;

    fprintf("\n---- TriRotor LQR run ----\n");
    fprintf("Hover trim F0 = [%.3f %.3f %.3f] N\n", F0_hover(1), F0_hover(2), F0_hover(3));
    fprintf("gain_scale_final = %.3f, ramp_time = %.2f s, zero_sum_dF = %d, invert_LM = %d\n", ...
        gain_scale_final, ramp_time, zero_sum_dF, invert_LM);

    %% === JSBSim I/O sockets ===
    % UDP receive: state from JSBSim
    u_rx = udpport("datagram", "IPV4", ...
                   "LocalPort", 5501, ...
                   "Timeout",   1.0);   % 1 s timeout

    % TCP send: thrust commands to JSBSim
    jsb = tcpclient("127.0.0.1", 5502, "Timeout", 1.0);
    pause(0.1);   % let JSBSim settle

    %% === Logging preallocation ===
    maxSteps = ceil(100 * Tsim);   % plenty for 50 Hz
    t_log = nan(maxSteps, 1);
    x_log = nan(maxSteps, 6);      % [p q r phi theta psi]
    F_log = nan(maxSteps, 3);      % [F1 F2 F3]
    k = 0;

    %% === State references & time markers ===
    x_ref   = [];   % reference attitude (set from first sample)
    simt0   = [];   % first JSBSim sim-time
    printed_init = false;

    fprintf("Running closed-loop LQR (UDP recv, TCP send)...\n");

    %% === Main loop ===
    while true

        % Wait for at least one datagram
        if u_rx.NumDatagramsAvailable == 0
            pause(0.005);
            continue;
        end

        % Read exactly one packet as a string datagram
        pkt = read(u_rx, 1, "string");  % returns a datagram object array
        if isempty(pkt)
            continue;
        end

        s = strtrim(pkt.Data);   % .Data is the string payload
        if strlength(s) == 0
            continue;
        end

        % Handle either comma-separated or whitespace-separated
        if contains(s, ",")
            parts = split(s, ",");
        else
            parts = split(s);    % split on whitespace
        end

        vals = str2double(parts);
        vals = vals(:);

        % Need at least [simt, p, q, r, phi, theta, psi]
        if numel(vals) < 7 || any(isnan(vals(1:7)))
            continue;
        end

        simt = vals(1);
        x    = vals(2:7);   % [p q r phi theta psi], all in rad/sec & rad
        x    = x(:);

        % Wrap yaw into [-pi, pi] just in case
        psi = x(6);
        x(6) = atan2(sin(psi), cos(psi));

        % Establish time origin and ref attitude on first good packet
        if isempty(simt0)
            simt0 = simt;
        end

        if isempty(x_ref)
            x_ref = x;             % reference attitude = initial attitude
            x_ref(1:3) = 0;        % but we want p,q,r -> 0

            fprintf("Ref attitude: phi=%.3f deg, theta=%.3f deg, psi=%.3f deg\n", ...
                rad2deg(x_ref(4)), rad2deg(x_ref(5)), rad2deg(x_ref(6)));
            fprintf("Initial x = [p q r phi theta psi] = [%g %g %g %g %g %g]\n", x);
        end

        % Stop condition: JSBSim sim time span
        if (simt - simt0) >= Tsim
            break;
        end

        %% === Error state and LQR torque ===
        x_tilde = x - x_ref;          % 6x1
        tau_LQR = -K_rot * x_tilde;   % 3x1

        % Optional sign flip on roll/pitch to match JSBSim conventions
        tau = tau_LQR;
        if invert_LM
            tau(1:2) = -tau(1:2);
        end

        % Gain ramp based on sim time
        t_rel = simt - simt0;
        gain_scale = gain_scale_final * min(1.0, max(0.0, t_rel / ramp_time));
        tau = gain_scale * tau;

        % Torque saturation
        tau(1) = max(-tau_max_roll_pitch, min(tau_max_roll_pitch, tau(1))); % roll
        tau(2) = max(-tau_max_roll_pitch, min(tau_max_roll_pitch, tau(2))); % pitch
        tau(3) = max(-tau_max_yaw,        min(tau_max_yaw,        tau(3))); % yaw

        %% === Map tau -> rotor thrusts ===
        % tau ≈ M_moments * dF, so dF = B_inv * tau
        dF = B_inv * tau;    % 3x1

        % Optionally force sum(dF)=0 so total thrust stays ~constant
        if zero_sum_dF
            dF = dF - mean(dF);
        end

        % Total thrusts
        F = F0_hover + dF;   % 3x1

        % Saturate rotor thrusts
        F = max(Fmin, min(Fmax, F));

        % Debug print every ~0.5s
        if ~printed_init || mod(k, 25) == 0
            fprintf(['DBG: t=%.3f  |x_tilde|=%.3g  gain=%.3f  ', ...
                     'tau=[%.3g %.3g %.3g], dF=[%.3g %.3g %.3g], F=[%.3g %.3g %.3g]\n'], ...
                    simt, norm(x_tilde), gain_scale, ...
                    tau(1), tau(2), tau(3), ...
                    dF(1), dF(2), dF(3), ...
                    F(1), F(2), F(3));
            printed_init = true;
        end

        %% === Send thrusts to JSBSim over TCP ===
        cmdStr = sprintf( ...
            ['set fcs/rotor1-thrust %.6f\n', ...
             'set fcs/rotor2-thrust %.6f\n', ...
             'set fcs/rotor3-thrust %.6f\n'], ...
             F(1), F(2), F(3));

        write(jsb, uint8(cmdStr));

        %% === Log ===
        k = k + 1;
        if k > maxSteps
            break;
        end

        t_log(k)   = simt;
        x_log(k,:) = x.';
        F_log(k,:) = F.';
    end

    % Trim logs
    t_log = t_log(1:k);
    x_log = x_log(1:k, :);
    F_log = F_log(1:k, :);

    fprintf("Run complete: %d samples logged.\n", k);
end
