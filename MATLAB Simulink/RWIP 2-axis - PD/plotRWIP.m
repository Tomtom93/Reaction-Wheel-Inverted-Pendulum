%% Plot Simulations
%convert rad/s to rpm by *30/pi
%convert rad to deg by *180/pi

figure('Name', 'Thetap');
plot(thetap_x.time, thetap_x.signals.values*180/pi,'-', thetap_y.time, thetap_y.signals.values*180/pi,'--');
title('Tilt angle of the pendulum : \theta_p');
xlabel('Time, t [s]');
ylabel('Pendulum angle, \theta_p [deg]');
legend('\theta_p x-axis','\theta_p y-axis');

figure('Name', 'dTheta_w');
plot(thetawdot_x.time, thetawdot_x.signals.values*30/pi,'-', thetawdot_y.time, thetawdot_y.signals.values*30/pi,'--'); 
title('Angular velocity of Reaction Wheels');
xlabel('Time, t [s]');
ylabel('Angular Velocity, d\theta_w [rpm]');
legend('d\theta_w x-axis','d\theta_w y-axis');

figure('Name', 'Torque');
plot(torque_x.time, torque_x.signals.values,'-', torque_y.time, torque_y.signals.values,'--');
title('Torque applied to Reaction Wheels');
xlabel('Time, t [s]');
ylabel('Motor Torque, \tau [N.m]');
legend('\tau x-axis','\tau y-axis');

figure('Name', 'dTheta_p');
plot(thetapdot_x.time, thetapdot_x.signals.values*180/pi,'-', thetapdot_y.time, thetapdot_y.signals.values*180/pi,'--');
title('Angular velocity of Pendulum Body');
xlabel('Time, t [s]');
ylabel('Angular Velocity, d\theta_p [deg/s]');
legend('d\theta_p x-axis','d\theta_p y-axis');

%% Plot Simulations Subplot

figure('Name', 'Thetap');
subplot(2,1,1);
plot(thetap_x.time, thetap_x.signals.values*180/pi, thetap_y.time, thetap_y.signals.values*180/pi);
title('Tilt angle of the pendulum : \theta_p');
xlabel('Time, t [s]');
ylabel('Pendulum angle, \theta_p [deg]');
legend('\theta_p x-axis','\theta_p y-axis');


%figure('Name', 'Torque');
subplot(2,1,2);
plot(torque_x.time, torque_x.signals.values, torque_y.time, torque_y.signals.values);
title('Torque applied to Reaction Wheels');
xlabel('Time, t [s]');
ylabel('Motor Torque, \tau [N.m]');
legend('\tau x-axis','\tau y-axis');

