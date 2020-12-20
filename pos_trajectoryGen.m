%% --------------------------DRIVER---------------------------------------
%{
Alonso Vega 
December 14, 2020


%}

%%
function r_ref = pos_trajectoryGen(t, x_0, y_0 , r)
    
    theta = linspace(0,2*pi,length(t));
    x     = r*cos(theta) + x_0;
    y     = r*sin(theta) + y_0;

    r_ref = [x', y'];
end 