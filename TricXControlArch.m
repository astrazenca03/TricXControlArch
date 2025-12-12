clc; clear;
disp("Control Architecture of Underactuated TriCopter");

x = zeros(1,3);
y = zeros(1,3);
for i = 1:3
    x(i) = input(sprintf("x%d=", i));
    y(i) = input(sprintf("y%d=", i));
end

disp("Enter spin directions(+1 up, -1 down):");
spin = zeros(1,3);
for i = 1:3
    spin(i) = input(sprintf("spin direction for x%d (1 or -1): ", i));
end

disp("Enter full inertia tensor (3x3):");
I = zeros(3,3);
for r = 1:3
    for c = 1:3
        I(r, c) = input(sprintf("I(%d,%d)=", r, c));
    end
end

disp("Enter diagonal 1 (6 states):");
Q = zeros(6,6);
for i = 1:6
    Q(i,i) = input(sprintf("Q(%d,%d)=", i, i));
end

disp("Enter diagonal R (4 inputs):");
R = zeros(4,4);
for j = 1:4
    R(j,j) = input(sprintf("R(%d,%d)=", j, j));
end

[M_full, M_moments, A, B, K] = mixer(x, y, spin, I, Q, R);

M_pinv = pinv(M_full);

disp("FULL MXER");
disp(M_full);

disp("MOMENT MIXER");
disp(M_moments);

disp(" Matrix A");
disp(A);

disp("Matrix B");
disp(B);

disp("Gain Vector K");
disp(K);

disp("REAL MiXER");
disp(M_pinv);

stability = A - B * K;
lambda = eig(stability);
disp("Eigenvalues");
disp(lambda);
if all(real(lambda) < 0)
    disp("System is stable.");
else
    disp("System is unstable.");
end

t = 0:0.001:20;
sys = ss(stability, zeros(6,1), eye(6), zeros(6, 1));
x0 = [0.3; -0.2; 0.15; 10*pi/180; -8*pi/180; 6*pi/180];

[y_out, t_out, x_out] = initial(sys, x0, t);

figure;
plot(t_out, x_out, "LineWidth", 1.2);
xlabel("Time(s)");
ylabel("Response");
legend("w_x", "w_y", "w_z", "phi", "theta", "psi");
grid on;
title("Dynamic Stability Check");

%%===Functions===

function [M_full, M_moments, A, B, K] = mixer(x, y, spin, I, Q, R)

N = length(x);

thrust = ones(1,N);
roll = y;
pitch = -x;
yaw = spin;

M_full = [thrust; roll; pitch; yaw];
M_moments = [roll; pitch; yaw];

A = zeros(6);
A(4,1) = 1;
A(5,2) = 1;
A(6,3) = 1;

Iinv = inv(I);

B = zeros(6,4);
B(1,2:4) = Iinv(1,:);
B(2,2:4) = Iinv(2,:);
B(3,2:4) = Iinv(3,:);

B_rot = zeros(6,3);
B_rot(1,1:3) = Iinv(1,:);
B_rot(2,1:3) = Iinv(2,:);
B_rot(3,1:3) = Iinv(3,:);

R_rot = diag([R(2,2), R(3,3), R(4,4)]);

[P_rot,~,K_rot] = care(A,B_rot,Q,R_rot);

K_full = zeros(4,6);
K_full(2:4,:) = K_rot;

K = K_full;
end
