figure;
plot(t_log, x_log(:,1:3), 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Angular rates [rad/s]');
legend('p','q','r'); grid on; title('Body rates');

figure;
plot(t_log, x_log(:,4:6)*180/pi, 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Angles [deg]');
legend('\phi','\theta','\psi'); grid on; title('Euler angles');

figure;
plot(t_log, F_log, 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Thrust [N]');
legend('F1','F2','F3'); grid on; title('Rotor thrusts');
