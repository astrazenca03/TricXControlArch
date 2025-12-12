clc; clear;

% === Geometry (must match TriRotor.xml) ===
x    = [0.185, -0.160,  0.045];   % [m]
y    = [0.125, -0.150,  0.215];   % [m]
spin = [1,    -1,       1   ];    % +1 / -1 for yaw torque

% === Mass & total hover thrust (must match XML) ===
m = 1.72;              % kg
g = 9.81;              % m/s^2
T_total = m * g;       % total thrust to support weight [N]

% === Inertia tensor (same as XML) ===
I = [ 0.0149  -0.0018   0.0004;
     -0.0018   0.0167  -0.0002;
      0.0004  -0.0002   0.0284];

% === LQR weighting (rotational states only are really used) ===
% x = [p q r phi theta psi]
Q = diag([25 25 25 20 20 20]);    % penalise rates + attitudes
R = diag([0.1 0.5 0.5 0.5]);      % only R(2:4,2:4) used

% === Build mixer / moment matrix ===
N = numel(x);

thrust = ones(1, N);     % total thrust row
roll   = y;              % L = sum( y_i * F_i )
pitch  = -x;             % M = sum(-x_i * F_i )
yaw    = spin;           % N = sum( s_i * F_i ), simple model

M_full    = [thrust; roll; pitch; yaw];   % 4x3
M_moments = [roll; pitch; yaw];           % 3x3

% This maps rotor thrusts to body moments (approx):
%   tau = M_moments * F
%
% For small *changes* around trim:
%   tau ≈ M_moments * dF  ->  dF = B_inv * tau
B_inv = inv(M_moments);

% === Compute proper hover trim F0 ===
% We want:
%   roll:  y * F0   = 0
%   pitch: -x * F0  = 0
%   thrust: sum(F0) = T_total
A_trim = [ y;
          -x;
           ones(1, N) ];

b_trim = [0; 0; T_total];

F0_hover = A_trim \ b_trim;   % 3x1

% check: equilibrium moments
tau_eq = M_moments * F0_hover;

disp('---- Design results ----');
disp('K_rot (before saving) will be computed below.');
disp('B_inv = ');
disp(B_inv);
disp('F0_hover = ');
disp(F0_hover.');
disp('Equilibrium tau_eq = [L M N] (should have L,M ~ 0):');
disp(tau_eq.');

% === Rotational state-space (simple integrator chain) ===
% x = [p q r phi theta psi]
A = zeros(6);
A(4,1) = 1;   % phi_dot   = p
A(5,2) = 1;   % theta_dot = q
A(6,3) = 1;   % psi_dot   = r

Iinv = inv(I);

% B_rot: how tau acts on rates (p,q,r)
% p_dot = Iinv(1,:) * tau, etc.
B_rot = zeros(6,3);
B_rot(1,1:3) = Iinv(1,:);
B_rot(2,1:3) = Iinv(2,:);
B_rot(3,1:3) = Iinv(3,:);

% Use only the rotational part of R
R_rot = diag([R(2,2), R(3,3), R(4,4)]);

% === LQR for rotational subsystem ===
[~,~,K_rot] = care(A, B_rot, Q, R_rot);   % K_rot: 3x6

disp('K_rot = ');
disp(K_rot);

% === Save for runtime ===
% === Save for runtime ===
F0 = F0_hover(:);      % make sure it's 3x1

save("trirotor_gains.mat", ...
     "K_rot", ...      % 3x6 LQR gain
     "B_inv", ...      % 3x3 torque->dF inverse
     "F0_hover");      % 3x1 hover thrust


disp("LQR design complete.");
