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
t_2     = 15.0;
delta_t = 0.01;
q_0     = [0; ...
           0; ...
           deg2rad(30)];

car        = Robot(q_0, t_1, t_2, delta_t);
t          = car.timeSpace;
closedLoop = true;

%% Controller 
if ~closedLoop
    % Open-Loop Controller 
    uTilda   = openLoop_control(t);
    phiTilda = uTilda(:,2);
    vTilda   = uTilda(:,1);

    car = car.set_control(uTilda);
else
    % Feedback Controller 
    r_ref = pos_trajectoryGen('l', t); 
    car   = car.set_reference(r_ref);
end

%% Get State Trajectory 
car       = car.solve(closedLoop);

%% Get States
qTilda    = car.trajectory(1:end-1,:);
qTildaHat = car.filteredTrajectory(1:end-1,:);
yTilda    = car.measurements(1:end-1,:);

% Get Control 
uTilda_actual   = car.controlTrajectory(1:end-1,:);
phiTilda_actual = uTilda_actual(:,2);
vTilda_actual   = uTilda_actual(:,1);

%% Plot State Trajectory wrt time
figure
subplot(3,2,1)
plot(t(1:end-1), qTilda(:,1), 'LineWidth', 2.5, 'Color', [0,0,0]);
hold on
scatter(t(1:end-1), yTilda(:,1), 'MarkerEdgeColor', [0,1,0],'MarkerFaceColor', [0,1,0]);
plot(t(1:end-1), qTildaHat(:,1), 'LineWidth', 0.5, 'Color', [1,0,0]);
titulo = title('x(t)');
titulo.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[m]', 'FontSize',13)
grid on
hold off

subplot(3,2,3)
plot(t(1:end-1), qTilda(:,2), 'LineWidth', 2.5, 'Color', [0,0,0]);
hold on
scatter(t(1:end-1), yTilda(:,2), 'MarkerEdgeColor', [0,1,0],'MarkerFaceColor', [0,1,0]);
plot(t(1:end-1), qTildaHat(:,2), 'LineWidth', 0.5, 'Color', [1,0,0]);
titulo = title('y(t)');
titulo.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[m]', 'FontSize',13)
grid on
hold off

subplot(3,2,5)
plot(t(1:end-1), rad2deg(qTilda(:,3)), 'LineWidth', 2.5, 'Color', [0,0,0]);
hold on
scatter(t(1:end-1), rad2deg(yTilda(:,3)), 'MarkerEdgeColor', [0,1,0],'MarkerFaceColor', [0,1,0]);
plot(t(1:end-1), rad2deg(qTildaHat(:,3)), 'LineWidth', 0.5, 'Color', [1,0,0]);
titulo = title('θ(t)');
titulo.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[°]', 'FontSize',13)
grid on
hold off

subplot(3,2,2)
plot(t(1:end-1), qTilda(:,1) - qTildaHat(:,1), 'LineWidth', 0.5, 'Color', [1,0,0]);
hold on
titulo = title('e_x(t)');
titulo.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[m]', 'FontSize',13)
grid on
hold off

subplot(3,2,4)
plot(t(1:end-1), qTilda(:,2) - qTildaHat(:,2), 'LineWidth', 0.5, 'Color', [1,0,0]);
hold on
titulo = title('e_y(t)');
titulo.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[m]', 'FontSize',13)
grid on
hold off

subplot(3,2,6)
plot(t(1:end-1), rad2deg(qTilda(:,3) - qTildaHat(:,3)), 'LineWidth', 0.5, 'Color', [1,0,0]);
hold on
titulo = title('e_θ(t)');
titulo.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[°]', 'FontSize',13)
grid on
hold off

%% Plot Positional Trajectory
figure 
plot(qTilda(:,1), qTilda(:,2), 'LineWidth', 2.5, 'Color', [0,0,0])
hold on
plot(car.referenceTrajectory(:,1), car.referenceTrajectory(:,2), '--r', 'LineWidth',1.0)
titulo = title('Car Trajectory');
titulo.FontSize = 15.0;
xlabel('x [m]', 'FontSize',13)
ylabel('y [m]', 'FontSize',13)
grid on


%% Plot Control
figure
subplot(2,1,1)
plot(t(1:end-1), rad2deg(phiTilda_actual), 'LineWidth', 1.75, 'Color', [1,0,0]);
hold on
titulo = title('φ(t)  (steering angle)');
titulo.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[°]', 'FontSize',13)
grid on
if ~closedLoop
    plot(t(1:end-1), rad2deg(phiTilda), 'LineWidth', 2.5, 'Color', [0,0,0]);
    legend('I/P Actuation', 'Actual')
end
hold off

subplot(2,1,2)
plot(t(1:end-1), vTilda_actual, 'LineWidth', 1.75, 'Color', [1,0,0]);
hold on
titulo = title('v(t)   (lateral speed)');
titulo.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[m/s]', 'FontSize',13)
grid on
if ~closedLoop
    plot(t(1:end-1), vTilda, 'LineWidth', 2.5, 'Color', [0,0,0]);
end 
hold off 



%%


