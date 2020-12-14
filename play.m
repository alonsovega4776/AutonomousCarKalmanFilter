%% ----------------------------------PLAY ANIME-------------------------------------------
%{
Alonso Vega 
December 14, 2020


%}

%% Paramters
loops = length(t);
frame(loops) = struct('cdata',[],'colormap',[]);

%% Play Figures
figure
for i = 1 : loops
    plot1config(car, i);
    
    frame(i) = getframe;
    clf;
end
%%