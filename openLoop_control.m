%% ------------------Control Trajectory Generator-------------------------------
%{
Alonso Vega 
December 12, 2020


%}

%%
function uTilda = openLoop_control(t)
%% Initialize
N = length(t);

vTilda   = zeros(N,1);
phiTilda = zeros(N,1);

%% Configure
%..........................................................................
% steering
t_phi = [1.0, 2.0;...
         3.0, 4.0];     %[s]
phi   = [20.0;...
         0.0];
phi = deg2rad(phi);     %[rad]

% speed
t_v = [1.0, 5.0;...
       6.0, 8.0];       %[s]
v   = [5.0;...
       -2.5];            %[m/s]
%..........................................................................
%% Generate Trajectory 
n_config_phi = length(phi);
for i = 1 : n_config_phi
    subDomain = (t >= t_phi(i,1)) & (t <= t_phi(i,2));
    phiTilda = phiTilda + subDomain*phi(i);
end

n_config_v = length(v);
for i = 1 : n_config_v
    subDomain = (t >= t_v(i,1)) & (t <= t_v(i,2));
    vTilda = vTilda + subDomain*v(i);
end

uTilda = [vTilda phiTilda];
end