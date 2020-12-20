%% ---------------------------------Animation---------------------------------------------
%{
Alonso Vega 
December 13, 2020


%}
%%
function plot1config(car, k)
%% Config
q = car.trajectory(k, :)';
u = car.controlTrajectory(k, :)';
L = car.wheelBase;
l = car.track;

w = car.tireWidth;
D = car.tireDiameter;

x     = q(1);
y     = q(2);
theta = q(3);

phi = u(2);

R_phi   = [cos(phi) -sin(phi) 0; ...
           sin(phi)  cos(phi) 0; ...
             0          0     1];

%% Plot Trajectory
subplot(1,4,1)
set(gcf,'Position',[0 1000 2000 500])
plot(car.trajectory(1:k,1), car.trajectory(1:k,2), 'LineWidth',2.5);
hold on
plot(car.referenceTrajectory(:,1), car.referenceTrajectory(:,2), '--r', 'LineWidth',1.0);

%% Vars 
x_vect_mid = [x;
              x + L*cos(theta)];
y_vect_mid = [y;
              y + L*sin(theta)];

x_vectR = x_vect_mid + 0.5*l*sin(theta);
y_vectR = y_vect_mid - 0.5*l*cos(theta);

x_vectL = x_vect_mid - 0.5*l*sin(theta);
y_vectL = y_vect_mid + 0.5*l*cos(theta); 

%% Rear Left Tire
x_tireL = [x_vectL(1);...
           x_vectL(1)-w*sin(theta);...
           x_vectL(1)+D*cos(theta)-w*sin(theta);...
           x_vectL(1)+D*cos(theta)];
y_tireL = [y_vectL(1);...
           y_vectL(1)+w*cos(theta);...
           y_vectL(1)+D*sin(theta)+w*cos(theta);...
           y_vectL(1)+D*sin(theta)];
patch(x_tireL, y_tireL, [0.1,0.1,0.1])

%% Rear Right Tire 
x_tireR = [x_vectR(1);...
           x_vectR(1)+w*sin(theta);...
           x_vectR(1)+D*cos(theta)+w*sin(theta);...
           x_vectR(1)+D*cos(theta)];
y_tireR = [y_vectR(1);...
           y_vectR(1)-w*cos(theta);...
           y_vectR(1)+D*sin(theta)-w*cos(theta);...
           y_vectR(1)+D*sin(theta)];
patch(x_tireR, y_tireR, [0.1,0.1,0.1])

%% Front Right Tire
x_tireR = [x_vectR(2);...
           x_vectR(2)+w*sin(theta);...
           x_vectR(2)+D*cos(theta)+w*sin(theta);...
           x_vectR(2)+D*cos(theta)];
y_tireR = [y_vectR(2);...
           y_vectR(2)-w*cos(theta);...
           y_vectR(2)+D*sin(theta)-w*cos(theta);...
           y_vectR(2)+D*sin(theta)];
       
x_tireR = x_tireR - D*cos(theta);
y_tireR = y_tireR - D*sin(theta);

% Roatating Tire------------------------------ 
r_vect        = [x_tireR y_tireR];
COM_tireR     = ones(1,4)*r_vect;
COM_tireR     = COM_tireR/4;
r_vectCOMtire = r_vect - ones(4,1)*COM_tireR;
r_vectCOMtire = [r_vectCOMtire' ;...
                    zeros(1,4)   ];         
r_vectCOMtire = R_phi*r_vectCOMtire;
r_vect        = r_vectCOMtire + [COM_tireR' * ones(1,4); zeros(1,4)];

x_tireR = r_vect(1,:)';
y_tireR = r_vect(2,:)';
% Roatating Tire------------------------------ 

patch(x_tireR, y_tireR, [0.1,0.1,0.1])

%% Front Left Tire
x_tireL = [x_vectL(2);...
           x_vectL(2)-w*sin(theta);...
           x_vectL(2)+D*cos(theta)-w*sin(theta);...
           x_vectL(2)+D*cos(theta)];
y_tireL = [y_vectL(2);...
           y_vectL(2)+w*cos(theta);...
           y_vectL(2)+D*sin(theta)+w*cos(theta);...
           y_vectL(2)+D*sin(theta)];
       
x_tireL = x_tireL - D*cos(theta);
y_tireL = y_tireL - D*sin(theta);

% Roatating Tire------------------------------ 
r_vect        = [x_tireL y_tireL];
COM_tireL     = ones(1,4)*r_vect;
COM_tireL     = COM_tireL/4;
r_vectCOMtire = r_vect - ones(4,1)*COM_tireL;
r_vectCOMtire = [r_vectCOMtire' ;...
                    zeros(1,4)   ];
r_vectCOMtire = R_phi*r_vectCOMtire;
r_vect        = r_vectCOMtire + [COM_tireL' * ones(1,4); zeros(1,4)];

x_tireL = r_vect(1,:)';
y_tireL = r_vect(2,:)';
% Roatating Tire------------------------------

patch(x_tireL, y_tireL, [0.1,0.1,0.1])

%% Chasis
line(x_vectR, y_vectR, 'color', [0,0,0],'LineWidth', 4);
line(x_vectL, y_vectL, 'color', [0,0,0],'LineWidth', 4);

patch([x_vectR(1) x_vectL(1) x_vectL(2) x_vectR(2)],...
      [y_vectR(1) y_vectL(1) y_vectL(2) y_vectR(2)], [0.5,0.5,0.5])

line([x_vectR(1) x_vectL(1)], [y_vectR(1) y_vectL(1)], 'color', [0,0,0],'LineWidth', 3);
line([x_vectR(2) x_vectL(2)], [y_vectR(2) y_vectL(2)], 'color', [0,0,0],'LineWidth', 3);
line(x_vect_mid, y_vect_mid, 'color', [0,0,0],'LineWidth', 0.2);

plot(x, y,'o','Markersize',5,'Markerface','r') 

%% Other
plot(COM_tireR(1), COM_tireR(2),'o','Markersize',2,'Markerface','g') 
plot(COM_tireL(1), COM_tireL(2),'o','Markersize',2,'Markerface','g') 

t = title(car.timeSpace(k));
t.FontSize = 15.0;

windowX = 2.5;
windowY = 2.5;
xlim([x-windowX, x+windowX]);
ylim([y-windowY, y+windowY]);
grid on
hold off



%% Plot Estimates 
qTilda    = car.trajectory(1:k,:);
qTildaHat = car.filteredTrajectory(1:k,:);
yTilda    = car.measurements(1:k,:);

t = car.timeSpace(1:k);

subplot(1,4,2)
plot(t, qTilda(:,1), 'LineWidth', 2.5, 'Color', [0,0,0]);
hold on
scatter(t, yTilda(:,1), 'MarkerEdgeColor', [0,1,0],'MarkerFaceColor', [0,1,0]);
plot(t, qTildaHat(:,1), 'LineWidth', 0.5, 'Color', [1,0,0]);
titulo = title('x(t)');
titulo.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[m]', 'FontSize',13)
grid on
hold off

subplot(1,4,3)
plot(t, qTilda(:,2), 'LineWidth', 2.5, 'Color', [0,0,0]);
hold on
scatter(t, yTilda(:,2), 'MarkerEdgeColor', [0,1,0],'MarkerFaceColor', [0,1,0]);
plot(t, qTildaHat(:,2), 'LineWidth', 0.5, 'Color', [1,0,0]);
titulo = title('y(t)');
titulo.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[m]', 'FontSize',13)
grid on
hold off

subplot(1,4,4)
plot(t, qTilda(:,3), 'LineWidth', 2.5, 'Color', [0,0,0]);
hold on
scatter(t, yTilda(:,3), 'MarkerEdgeColor', [0,1,0],'MarkerFaceColor', [0,1,0]);
plot(t, qTildaHat(:,3), 'LineWidth', 0.5, 'Color', [1,0,0]);
titulo = title('θ(t)');
titulo.FontSize = 15.0;
xlabel('t [s]', 'FontSize',13)
ylabel('[°]', 'FontSize',13)
grid on
hold off
end

%%