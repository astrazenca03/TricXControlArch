function F = stepTriRotor(x, K_rot, M_pinv, mass, Fmax)
    g      = 9.81;
    Ftotal = mass * g;
    F0     = (Ftotal/3) * ones(3,1);

    % LQR torque demand
    tau = -K_rot * x(:);          % 3x1

    % Map torques to thrust *increments*
    dF = M_pinv * tau;            % 3x1

    % Enforce zero-sum BEFORE adding to F0
    dF = dF - mean(dF);

    % Hover + increments
    F = F0 + dF;

    % Saturate at the very end
    F = max(min(F, Fmax), 0);
end
