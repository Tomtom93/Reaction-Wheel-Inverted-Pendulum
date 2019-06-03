%% Plot Simulations

figure('Name', 'Theta_p');
plot(theta_p.time, theta_p.signals.values*180/pi,'LineWidth',2);
title('Tilt angle of the pendulum : \theta_p');
xlabel('Time, t [s]');
ylabel('Pendulum angle in x-axis, \theta_p [deg]');
grid on