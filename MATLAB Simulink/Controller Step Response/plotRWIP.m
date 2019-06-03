%% Plot Simulations

figure('Name', 'Theta_p');
plot(thetap.time, thetap.signals.values,'LineWidth',2);
title('Step Response of Closed-loop under PID Control');
xlabel('Time, t [s]');
ylabel('Amplitude [rad]');
grid on

figure('Name', 'dTheta_p');
plot(thetapdot.time, thetapdot.signals.values*180/pi, 'LineWidth',2);
title('Open-loop Step Response');
xlabel('Time, t [s]');
ylabel('Amplitude, d\theta_p [deg/s]');
grid on

figure('Name', 'dTheta_w');
plot(thetawdot.time, thetawdot.signals.values*30/pi, 'LineWidth',2); 
title('Open-loop Step Response');
xlabel('Time, t [s]');
ylabel('Angular Velocity, d\theta_w [rpm]');
grid on