%% --------------------------DRIVER---------------------------------------
%{
Alonso Vega 
December 11, 2020


%}
clear all
clc
close all
%% Initialize 
t_1     = 0;
t_2     = 10.0;
delta_t = 0.01;
q_0     = [0; ...
           0; ...
           deg2rad(30)];

car = Robot(q_0, t_1, t_2, delta_t);
%% Get Control
t = car.timeSpace;

uTilda   = openLoop_control(t);
phiTilda = uTilda(:,2);
vTilda   = uTilda(:,1);

%% Set Control 
car = car.set_control(uTilda);

uTilda_actual = car.controlTrajectory;
phiTilda_actual = uTilda_actual(:,2);
vTilda_actual   = uTilda_actual(:,1);

%% Get State Trajectory 
car    = car.solve();
qTilda = car.trajectory; 


%% Plot State Trajectory wrt time
figure
subplot(3,1,1)
plot(t, qTilda(:,1), 'LineWidth', 2.5, 'Color', [0,0,0]);
hold on
t_phi = title('x(t)');
t_phi.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[m]', 'FontSize',13)
grid on
hold off

subplot(3,1,2)
plot(t, qTilda(:,2), 'LineWidth', 2.5, 'Color', [0,0,0]);
hold on
t_phi = title('y(t)');
t_phi.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[m]', 'FontSize',13)
grid on
hold off

subplot(3,1,3)
plot(t, qTilda(:,3), 'LineWidth', 2.5, 'Color', [0,0,0]);
hold on
t_phi = title('θ(t)');
t_phi.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[°]', 'FontSize',13)
grid on
hold off

%% Plot Positional Trajectory
figure 
plot(qTilda(:,1), qTilda(:,2), 'LineWidth', 2.5, 'Color', [0,0,0])
t_phi = title('Car Trajectory');
t_phi.FontSize = 15.0;
xlabel('x [m]', 'FontSize',13)
ylabel('y [m]', 'FontSize',13)
grid on


%% Plot Control
figure
subplot(2,1,1)
plot(t, rad2deg(phiTilda), 'LineWidth', 2.5, 'Color', [0,0,0]);
hold on
plot(t, rad2deg(phiTilda_actual), 'LineWidth', 1.75, 'Color', [1,0,0]);
t_phi = title('φ(t)  (steering angle)');
t_phi.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[°]', 'FontSize',13)
grid on
legend('I/P Actuation', 'Actual')
hold off

subplot(2,1,2)
plot(t, vTilda, 'LineWidth', 2.5, 'Color', [0,0,0]);
hold on
plot(t, vTilda_actual, 'LineWidth', 1.75, 'Color', [1,0,0]);
t_phi = title('v(t)   (lateral speed)');
t_phi.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[m/s]', 'FontSize',13)
grid on
hold off 


%% 


%%


