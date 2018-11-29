clear all, close all, clc

%INITIALIZE VARIABLES
N=10;                        % Agents per team
Offense = 5;
Defense = 5;

dt=0.01;                   % numerical steplength
max_iter = 2000;
center = [0;0];
videoFLag = 0;                          % Change to 1 to record video
circularInitialConditions = 0;          % Change to 0 for random initial condition (needed for Q2.d)
                           
% INITIALIZE ROBOTARIUM
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

xuni = r.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,:);                                            % x-y positions only
r.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
r.step();                                                % Run robotarium step
PopulateHazards(5,[0.03,0.2],[-1,1;-1,1]);
[si_to_uni_dyn] = create_si_to_uni_mapping3();
  

% AGENT LOGIC HERE!!!
for k = 1:max_iter
    
    % Get new data and initialize new null velocities
    xuni = r.get_poses();                                   % Get new robots' states
    x = xuni(1:2,:);                                        % Extract single integrator states
    u=zeros(2,N);                                           % Initialize velocities to zero

     for i= 1:N
         for j= 1:N     
         end
     end
     
    disp(k);
    dx = si_to_uni_dyn(u, xuni);                            % Convert single integrator inputs into unicycle inputs
    r.set_velocities(1:N, dx); r.step();                    % Set new velocities to robots and update
end

% experiment is over!
r.debug();




% FUNCTIONS HERE!!!
function PopulateHazards(number,sizes,arena)
    for k = 1:number
        sz = rand;
        width = arena(1,2)-arena(1,1);
        height = arena(2,2)-arena(1,1);
        loc = rand(2,1) - [0.5;0.5];
        loc = [width,0;0,height]*loc;
        
        result = (sizes(2) - sizes(1))*sz;
        actualSize = sizes(1) + result;
        CreateObstacle(actualSize,loc);
    end
end

function CreateObstacle(size,location)
P = [size, 0; 0, size];
center = location;
t = linspace(0,2*pi,30);
patch(P(1,1)*cos(t) + center(1), P(2,2)*sin(t) + center(2), [0.32,0.32,0.32]); %create a filled polygon - in this case an oval
end