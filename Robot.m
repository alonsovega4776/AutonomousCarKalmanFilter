%% --------------------------ROBOT---------------------------------------
%{
Alonso Vega 
December 12, 2020


%}

%% Class Definition 
classdef Robot
    %% PROPERTIES
    properties(GetAccess=private)        
        % Kalaman Variables 
        x_kHatPLUS_
        P_kPLUS_
              
    end 
        
    properties(GetAccess=public)  
        initalTime               {mustBeNumeric}
        finalTime                {mustBeNumeric}
        timeResolution           {mustBeNumeric}
        currentStep              {mustBeNumeric}
        timeSpace                {mustBeNumeric}
        
        config                   {mustBeNumeric}
        trajectory               {mustBeNumeric}
        referenceTrajectory      {mustBeNumeric}
        
        measurements 
        filteredTrajectory       {mustBeNumeric}
        controlTrajectory        {mustBeNumeric}
        
    end
    
    properties(Constant)
        %% Geometry 
        tireDiameter = 0.15;
        tireWidth    = 0.065;
        wheelBase    = 0.40;
        track        = 0.25; 
        
        % Controller Gains
        gainK_PI = [1.00 ;...
                    0.25]
        gainK_P  = 1.00
        
        % Limits
        uMAX = [      2.5  ;...
                deg2rad(20)]
        uMIN = -[     2.5  ;...
                 deg2rad(20)]
        
        % Statistics      
        measurementUncertainty = diag([5.0, 5.0, deg2rad(40.0)]);
        modelUncertainty       = diag([0.72, 1.515, 0.9]);
        
        % Enable Filter
        kalman_EN = true;
    end
    
    %% METHODS
    methods
        %% ++++++++++++++++++++++ Constructor ++++++++++++++++++++++
        
        function robot_object = Robot(q_1, t_initial, t_final, time_step)
            if nargin == 4
                robot_object.initalTime      = t_initial;
                robot_object.finalTime       = t_final;
                robot_object.timeResolution  = time_step;
                robot_object.currentStep     = 1;
                robot_object.timeSpace       = [t_initial : time_step : t_final]';
                
                robot_object.controlTrajectory   = zeros(length(robot_object.timeSpace), 2);
                robot_object.referenceTrajectory = zeros(length(robot_object.timeSpace), 2);
                
                robot_object.config        = q_1;
                robot_object.trajectory    = zeros(length(robot_object.timeSpace), 3);
                
                robot_object.measurements  = zeros(length(robot_object.timeSpace), 3); 
                robot_object.filteredTrajectory = zeros(length(robot_object.timeSpace), 3);
                
                robot_object.trajectory(1,:)    = q_1';
                robot_object.filteredTrajectory(1,:) = q_1';
                robot_object.measurements(1,:)  = q_1';  % Full-state Feedback
                
                robot_object.x_kHatPLUS_ = q_1;
                robot_object.P_kPLUS_    = zeros(3,3);
                
            else
                error('ERROR: Robot Constructor: Need 4 input arguments.');
                exit;
            end
        end
        
        %% ++++++++++++++++++++++ Solve for State Trajectory ++++++++++++++++++++++
        
        function obj = solve(obj, closedLoop)
            
            if closedLoop
                for k = 1: length(obj.timeSpace)-1
                    %% Get Control I/P
                    [obj, u_k] = obj.controller();
                    [obj, u_k] = obj.sat_control(u_k);
                    
                    obj.controlTrajectory(k,:) = u_k';
                    
                    %% New Configuration  
                    q_k                   = obj.config;          % actual config. or state
                    [obj, q_kPlus1]       = obj.kin_transitionActual(q_k, u_k);
                    
                    obj.config            = q_kPlus1;
                    obj.currentStep       = obj.currentStep + 1; 
                    obj.trajectory(k+1,:) = q_kPlus1;
                end
                
            else
                for k = 1: length(obj.timeSpace)
                    q_k                   = obj.config;
                    [~, q_kPlus1]         = obj.kin_transitionActual(q_k, obj.controlTrajectory(k,:)');
                    
                    obj.config            = q_kPlus1;
                    obj.currentStep       = obj.currentStep + 1; 
                    obj.trajectory(k+1,:) = obj.config;
                end 
            end
        end
        
        %% ++++++++++++++++++++++ Pure Persuit Controller ++++++++++++++++++++++
        
        function [obj, u_k] = controller(obj)
            %% Parameters
            epsilon = 0.1;
            k_PI    = obj.gainK_PI;
            k_P     = obj.gainK_P;
            
            k = obj.currentStep;
            %% Measurements 
            % State or Config. of k = 1 is exactly known (assumption)
            if k >= 2
                [~, y_k]               = obj.get_measurements();
                obj.measurements(k, :) = y_k';
    
                [obj, x_kFiltered]           = obj.extended_kalman_filter();
                if obj.kalman_EN
                    obj.filteredTrajectory(k, :) = x_kFiltered'; 
                else
                    obj.filteredTrajectory(k, :) = y_k'; 
                end
              
            end
            
            %% Errors
            t = obj.timeSpace;
            
            r_refTilda_k = obj.referenceTrajectory(1:k, :);
            r_Tilda_k    = obj.filteredTrajectory(1:k, 1:2);
            e_rTilda     = r_refTilda_k - r_Tilda_k;
            e_rTildaMag  = vecnorm(e_rTilda')' - epsilon;
                       
            r_ref_k = obj.referenceTrajectory(k,:)';
            q_k     = obj.filteredTrajectory(k,:)';
            r_k     = q_k(1:2);
            theta_k = q_k(3);
            e_r     = r_ref_k - r_k;
            e_rMag  = norm(e_r) - epsilon; 
            
            theta_ref_k = atan2(r_ref_k(2)-r_k(2), r_ref_k(1)-r_k(1));
            e_theta     = angdiff(theta_k, theta_ref_k);
     
            %% PI Control
            if k == 1
                e_integral = 0;
            else
                e_integral = trapz(t(1:k), e_rTildaMag);
            end
            u_v = k_PI'*[   e_rMag    ;...
                         e_integral]; 
            
            
            %% P Control
            u_theta = k_P*e_theta;
            
            %% O/P
            u_k = [u_v     ;...
                   u_theta];
        end
        
        %% ++++++++++++++++++++++ Measurements ++++++++++++++++++++++
        
        function [obj, y_k] = get_measurements(obj)
            H   = [1 0 0 ;...
                   0 1 0 ;...
                   0 0 1];
            R_k = obj.measurementUncertainty;
            
            v_k = mvnrnd([0; 0; 0], R_k, 1)'; % white noise 
            %% Observation
            x_k = obj.config;       % our state, x, is just the configuration, q
            y_k = H*x_k + v_k;      % only angle has noise  
            
        end
        
        %% ++++++++++++++++++++++ Kalman Filter ++++++++++++++++++++++
        
        function [obj, x_kHatPLUS] = extended_kalman_filter(obj)
            %% Initialize Filter
            k = obj.currentStep;
            
            R_k       = obj.measurementUncertainty;
            Q_kMinus1 = obj.modelUncertainty;
            
            x_kMinus1HatPLUS = obj.x_kHatPLUS_;
            P_kMinus1PLUS    = obj.P_kPLUS_;
            
            % Update state/control
            u_kMinus1 = obj.controlTrajectory(k-1,:)';
            y_k       = obj.measurements(k, :)'; 

            %% Compute F and L

            [~, F_kMinus1] = diff_transitionModel(obj, x_kMinus1HatPLUS, u_kMinus1);
            L_kMinus1      = eye(3);

            %% A Priori 
            P_kMINUS         = F_kMinus1*P_kMinus1PLUS*F_kMinus1' ...
                                    + L_kMinus1*Q_kMinus1*L_kMinus1'; 
            [~, x_kHatMINUS] = kin_transitionModel(obj, x_kMinus1HatPLUS, u_kMinus1);  

            %% Compute H, M, and Kalman Gain
            H_k = eye(3);
            M_k = eye(3);

            
            K_k = P_kMINUS*H_k';
            K_k = K_k/(H_k*P_kMINUS*H_k' + M_k*R_k*M_k');
            
            %% A Posteriori 
            % assume h(x) = x
            x_kHatPLUS = x_kHatMINUS + K_k*(y_k - x_kHatMINUS);
            P_kPLUS    = (eye(3,3) - K_k*H_k)*P_kMINUS;
            
            %% Update 
            obj.x_kHatPLUS_  = x_kHatPLUS;
            obj.P_kPLUS_     = P_kPLUS;

        end 
        
         %% +++++++++++++ Derivative of Transition Equation Model wrt State +++++++++++++++
        % For Kalman Filter
        function [obj, diff_f] = diff_transitionModel(obj, x, u)
            x_k     = x(1);
            y_k     = x(2);
            theta_k = x(3);
            
            v_k   = u(1);
            phi_k = u(2);
            
            delta_t = obj.timeResolution;
            
            SC_vect = [-sin(theta_k);...
                        cos(theta_k);...
                             0      ;];
            diff_f  = eye(3) + delta_t*v_k*[zeros(3,1) zeros(3,1) SC_vect];
        end
        %% ++++++++++++++++++++++ Transition Equation Model ++++++++++++++++++++++
        % For Kalman Filter
        function [obj, q_kPlus1] = kin_transitionModel(obj,q_k, u_k)
            %% Access Variables and Constants
            x_k     = q_k(1);
            y_k     = q_k(2);
            theta_k = q_k(3);
            
            v_k   = u_k(1);
            phi_k = u_k(2);
            
            model_error_1 = 0.01;
            model_error_2 = 0.85;
            model_error_3 = -deg2rad(10);
            
            L       = obj.wheelBase + model_error_1;
            delta_t = obj.timeResolution;
            
            %% Update
            S_k      = [cos(theta_k) sin(theta_k+model_error_3) tan(phi_k)/L ;...
                             zeros(1,3)                   ];
            q_kPlus1 = model_error_2*q_k + delta_t*S_k'*u_k;
            
        end
        
        %% ++++++++++++++++++++++ Actual Transition Equation ++++++++++++++++++++++
        
        function [obj, q_kPlus1] = kin_transitionActual(obj,q_k, u_k)
            %% Access Variables and Constants
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
            
        end
        
        %% ++++++++++++++++++++++ Clip Trajectory ++++++++++++++++++++++
        
        function [obj, new_uTilda] = sat_control(obj, uTilda)
            phi_Max = obj.uMAX(2);
            phi_Min = obj.uMIN(2);
            v_Max   = obj.uMAX(1);
            v_Min   = obj.uMIN(1);
            
            if size(uTilda, 1) > 2
                vTilda   = uTilda(:,1);
                phiTilda = uTilda(:,2);
            else
                vTilda   = uTilda(1);
                phiTilda = uTilda(2);
            end
            
            %% clip speed
            clip_Max = vTilda > v_Max;
            clip_Min = vTilda < v_Min;
            
            vTilda(clip_Max) = v_Max;
            vTilda(clip_Min) = v_Min;
            
            %% clip steering
            clip_Max = phiTilda > phi_Max;
            clip_Min = phiTilda < phi_Min;
            
            phiTilda(clip_Max) = phi_Max;
            phiTilda(clip_Min) = phi_Min;
            
            %% O/P
            new_uTilda = [vTilda, phiTilda];
            if size(uTilda, 1) == 2
                new_uTilda = new_uTilda';
            end
            
           
        end
        
        %% ++++++++++++++++++++++ Reference I/P for Feedback ++++++++++++++++++++++
        
        function obj = set_reference(obj, r_ref)
            if sum(size(obj.referenceTrajectory) == size(r_ref)) == 2
                obj.referenceTrajectory = r_ref;  
            else
                error('ERROR: invalid input reference configuration.');
                exit;
            end
            
            
        end
        %% ++++++++++++++++++++++ Control I/P ++++++++++++++++++++++
        function obj = set_control(obj, input)
            if sum(size(obj.controlTrajectory) == size(input)) == 2
                [~, input] = obj.sat_control(input);
                obj.controlTrajectory = input;  
            else
                error('ERROR: invalid input trajectory.');
                exit;
            end
        end 
        
    end
end 
%%

