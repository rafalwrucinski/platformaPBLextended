clear all;

%% Assign the xy-locations of the stations
mapSize = zeros(7,7);

myGoals = [  2.5   1  1;   ...
            0.2   1   0];
        
%% ktora czesc mapy chcemy widziec podczas symulacji
% mozna zmieniac w trakcie! [Xmax, Ymax, Xmin, Ymin];
vizLim=[5;5;0;0];

%% You can modify here: desired linear velocity (desired_vel), 
% linear'n'angular velocity (slow_vel/ slow_ang) while docking...
desired_vel= 0.4;
max_ang_vel=1;
slow_vel=0.2;
slow_ang=0.2;
distToReach=0.1;

%% Open Simulink model 
% open_system('simulinkPBLextended');