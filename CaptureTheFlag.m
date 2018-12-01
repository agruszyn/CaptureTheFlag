clear all, close all, clc

%INITIALIZE VARIABLES
N=10;                        % Agents per team
Obstacles = 7;               % Number of obstacles
ob = zeros(3,Obstacles);
init(1,1:N) = 2.5;
init(2,1:N) = [-0.9, -0.7, -0.5, -0.3, -0.1, 0.1, 0.3, 0.5, 0.7, 0.9];
init(3,1:N) = 0;
Offense = 5;
Defense = 5;

dt=0.01;                   % numerical steplength
max_iter = 2000;
center = [0;0];
videoFLag = 0;                          % Change to 1 to record video
circularInitialConditions = 0;          % Change to 0 for random initial condition (needed for Q2.d)
                           
% INITIALIZE ROBOTARIUM
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', init);

xuni = r.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,:);                                            % x-y positions only
r.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
r.step();                                                % Run robotarium step
ob = PopulateHazards(Obstacles,[0.05,0.3],[-1.5,1.5;-1.7,1.7]);
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
     
    %disp(k);
    dx = si_to_uni_dyn(u, xuni);                            % Convert single integrator inputs into unicycle inputs
    r.set_velocities(1:N, dx); r.step();                    % Set new velocities to robots and update
end

% experiment is over!
r.debug();




% FUNCTIONS HERE!!!
function y = PopulateHazards(number,sizes,arena)
fuck(1:3,1:number) = 999;
    for k = 1:number
        valid = 0;
        sz = rand;
        width = arena(1,2)-arena(1,1);
        height = arena(2,2)-arena(1,1);
        
        loc = rand(2,1) - [0.5;0.5];
        loc = [width,0;0,height]*loc;
        result = (sizes(2) - sizes(1))*sz;
        actualSize = sizes(1) + result;
        
        while valid < (k)
        loc = rand(2,1) - [0.5;0.5];
        loc = [width,0;0,height]*loc;
        valid = 0;
            for a = 1:k
                disp(valid);
                if  sqrt((fuck(2,a) - loc(1,1)).^2 + (fuck(3,a) - loc(2,1)).^2) > ((actualSize + fuck(1,a)) + 0.01) 
                    valid = valid + 1;
                    disp(sqrt((fuck(2,a) - loc(1,1)).^2 + (fuck(3,a) - loc(2,1)).^2) - (fuck(1,a) + actualSize));
                end
            end
        
        end    

        CreateObstacle(actualSize,loc);
        fuck(1,k) = actualSize;
        fuck(2:3,k) = loc;
        disp(fuck);
    end
    y = fuck;
end

function CreateObstacle(size,location)
P = [size, 0; 0, size];
center = location;
t = linspace(0,2*pi,30);
patch(P(1,1)*cos(t) + center(1), P(2,2)*sin(t) + center(2), [0.32,0.32,0.32]); %create a filled polygon - in this case an oval
end