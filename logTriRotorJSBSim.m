function [t_log, x_log] = logTriRotorJSBSim(Tsim)
    % Tsim in seconds

    % 1) Set up UDP listener
    u = udpport("datagram","IPV4","LocalPort",5501,"Timeout",2);
    disp("Listening on UDP 5501...");

    % 2) Preallocate (rough) – assume, say, 500 Hz
    maxSteps = ceil(500*Tsim);
    t_log = nan(maxSteps,1);
    x_log = nan(maxSteps,6);  % [p q r phi theta psi]

    % 3) Skip header line once
    gotHeader = false;
    k = 0;

    tic;
    while toc < Tsim
        pkt = read(u, 1, "string");

        if isempty(pkt)
            % nothing came in this cycle, just continue
            continue;
        end

        s = pkt.Data;

        % Header?
        if ~gotHeader && contains(s, "<LABELS>")
            gotHeader = true;
            disp("Header received, starting numeric logging...");
            continue;
        end

        % Parse numeric line
        parts = split(strtrim(s), ",");
        vals  = str2double(parts);

        if numel(vals) < 8 || any(isnan(vals))
            % malformed row, skip
            continue;
        end

        simt = vals(2);
        state = vals(3:8);   % [p q r phi theta psi]

        k = k+1;
        t_log(k,1)   = simt;
        x_log(k,1:6) = state;
    end

    % Trim NaNs
    t_log = t_log(1:k);
    x_log = x_log(1:k,:);

    % Cleanup
    clear u;
end
