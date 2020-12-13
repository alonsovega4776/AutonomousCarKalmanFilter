%% --------------------------ROBOT---------------------------------------
%{
Alonso Vega 
December 12, 2020


%}

%% Class Definition 
classdef Robot
    %% PROPERTIES
    properties(GetAccess=public)
        initalTime        {mustBeNumeric}
        finalTime         {mustBeNumeric}
        timeResolution    {mustBeNumeric}
        config            {mustBeNumeric}
        currentStep       {mustBeNumeric}
        trajectory        {mustBeNumeric}
        controlTrajectory {mustBeNumeric}
        timeSpace         {mustBeNumeric}
    end
    properties(Constant)
        wheelBase = 0.25;
        track     = 0.05; 
        uMAX      = [2.0;...
                     deg2rad(25)]
        uMIN      = -[2.0;...
                     deg2rad(25)]
    end
    
    %% METHODS
    methods
        %% Constructor 
        function robot_object = Robot(q_0, t_initial, t_final, time_step)
            if nargin == 4
                robot_object.initalTime        = t_initial;
                robot_object.finalTime         = t_final;
                robot_object.timeResolution    = time_step;
                robot_object.config            = q_0;
                robot_object.currentStep       = 0;
                robot_object.timeSpace         = [t_initial : time_step : t_final]';
                robot_object.controlTrajectory = zeros(length(robot_object.timeSpace), 2);
                robot_object.trajectory        = zeros(length(robot_object.timeSpace), 2);
            else
                error('ERROR: Robot Constructor: Need 4 input arguments.');
                exit;
            end
        end
        
        %% Transition Equation 
        function obj = kin_transition(obj, u_k)
            %% Access Variables and Constants
            q_k = obj.config;
            
            x_k     = q_k(1);
            y_k     = q_k(2);
            theta_k = q_k(3);
            
            v_k   = u_k(1);
            phi_k = u_k(2);
            
            L       = obj.wheelBase;
            delta_t = obj.timeResolution;
            
            %% Update
            S_k      = [cos(theta_k) sin(theta_k) tan(phi_k)/L ;...
                             zeros(1,3)                   ];
            q_kPlus1 = q_k + delta_t*S_k'*u_k;
            
            obj.config        = q_kPlus1;
            obj.currentStep   = obj.currentStep + 1; 
        end
        
        %% Trajectory 
        function obj = get_trajectory(obj)
            uTilda  = obj.controlTrajectory;
            qTilda  = object.trajectory;
            
            for k = 1:length(t)
                u_k         = uTilda(k,:);
                qTilda(k,:) = (obj.kin_transition(u_k).config)';              
            end
            
            obj.controlTrajectory = qTilda;
        end
        
        %% Control Saturation
        function obj = set_control(obj, input)
            if sum(size(obj.controlTrajectory) == size(input)) == 2
                obj.controlTrajectory = input;  
            else
                error('ERROR: invalid input trajectory.');
                exit;
            end
        end 
        
    end
end 
%%