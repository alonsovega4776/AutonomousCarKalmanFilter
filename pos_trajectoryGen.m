%% --------------------------DRIVER---------------------------------------
%{
Alonso Vega 
December 14, 2020


%}

%%
function r_ref = pos_trajectoryGen(shape, t)
    x_0  = 0.5; 
    y_0  = 0.5; 
    r    = 2.0;
    
    m = 2.0;
    b = 1.0;
    
    if shape == 'c'
        theta = linspace(0,2*pi,length(t));
        x     = r*cos(theta) + x_0;
        y     = r*sin(theta) + y_0;
    elseif shape == 'l'
        
        y = m*t + b;
        x = t;
        
        x = x';
        y = y';
        
    end
    
    

    r_ref = [x', y'];
end 