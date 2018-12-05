clear all, close all, clc

%(------------------------------------------ PARAMETERS ------------------------------------------)
%%
hazardTotal = 7;                            % Total number of obstacles
hazardSizes = [0.05,0.3];
hazardWindow = [-1,1;-1.8,1.8];

flagSize = 0.25;
redFlagpos = [2,1.1];
blueFlagpos = [-2,-1.1];
perimeterSize = 0.4;

N = 10;                                     % Total number of players team
init(1,1:N) = 2.5;                          % starting X position
init(2,1:N) = linspace(1.5,-1.5,10);        % starting Y position
init(3,1:N) = 2;                            % starting rotation
viewingDistance = 2;                        % Range of vision for each player
D = [1,5];                                  % Players on Defense
O = [6,10];                                 % Players on Offense

max_iter = 3000;                            % max script time
dt = 0.01;                                  % numerical steplength

lambda = 0.05;                              % lambda for barrier certificate
barrierDistance = 1.2;                       % safety distance for barrier certificate
%%
%(------------------------------------------ INITIALIZATIONS ------------------------------------------)
%%
%INITIALIZE VARIABLES
hazardProperties = zeros(3,hazardTotal);    % Location and size of hazards
center = [0;0];                             % center of field
A = zeros(N,N);                             % agent connection matrix
th = 0:pi/50:2*pi;                          % list of points around a circle

% INITIALIZE ROBOTARIUM
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', init);     % Robotarium initial conditions
safety = barrierDistance * r.robot_diameter; 

xuni = r.get_poses();                                       % States of real unicycle robots
x = xuni(1:2,:);                                            % x-y positions only
r.set_velocities(1:N, zeros(2,N));                          % Assign dummy zero velocity
r.step();                                                   % Run robotarium step
hazardProperties = PopulateHazards(hazardTotal, hazardSizes, hazardWindow);     % create the hazards

CreateFlag(flagSize, redFlagpos, 'red');                                % create the red flag
CreateFlag(flagSize, blueFlagpos, 'blue');                              % create the blue flag

xunit = viewingDistance * cos(th) + x(1,10);                    % x position of points around a circle
yunit = viewingDistance * sin(th) + x(2,10);                    % y position of points around a circle
h = plot(xunit, yunit);                                         % plot the circle
h.XDataSource = 'xunit'; h.YDataSource = 'yunit';               % source for updating circle position


[si_to_uni_dyn] = create_si_to_uni_mapping2();                  % unicycle mapping
% Grab barrier certificates for unicycle dynamics
uni_barrier_cert = create_uni_barrier_certificate('SafetyRadius', safety, 'ProjectionDistance', lambda);    % barrier mapping
%%
%(------------------------------------------ UPDATE FUNCTION ------------------------------------------)
%%
for k = 1:max_iter
    disp(k);
    xuni = r.get_poses();                                   % Get new robots' states
    u = zeros(2,N);
    
    % group up together
    if k < 700
    u(:,D(1):D(2)) = Huddle(perimeterSize, xuni, D, redFlagpos);                 % Defense huddle
    u(:,O(1):O(2)) = Huddle(perimeterSize, xuni, O, (redFlagpos - [0,2]));       % Offense huddle
    end
    if k > 700
    u(:,D(1):D(2)) = CircleAround(xuni, D, A, redFlagpos, perimeterSize);
    end
    %update the circle
    [xunit, yunit] = UpdateCircle(xuni(1:2,:), th, viewingDistance);
    refreshdata(h);
    
    %update the edges
    A = UpdateConnections(A,viewingDistance, x, N);
    
    %move robots
    dx = si_to_uni_dyn(u, xuni);                            % Convert single integrator inputs into unicycle inputs
    dx = uni_barrier_cert(dx, xuni);
    r.set_velocities(1:N, dx); r.step();                    % Set new velocities to robots and update
end

% experiment is over!
r.debug();
%%
%(------------------------------------------ FUNCTIONS ------------------------------------------)
%%
function y = FollowTheLeader(xuni, players, A)

end

function y = CircleAround(xuni, players, A, center, size)
teamSize = players(2) - players(1) + 1;
x = xuni(1:2,players(1):players(2)); % Extract single integrator states
K =  size / (sqrt([1,1] * (transpose(center) - x(:,1)).^2));
R = [cos(K*pi/teamSize) sin(K*pi/teamSize); -sin(K*pi/teamSize) cos(K*pi/teamSize)];
u = zeros(2,teamSize);                                                    % Initialize velocities to zero
L = CreateCircleGraph(A(players(1):players(2),players(1):players(2)));
    for i= 1:teamSize
         for j= 1:teamSize  
             if j ~= i
                u(:,i) = u(:,i) + 0.3 * L(i,j) * (x(:,j)-x(:,i));
             end
         end
     
    end
    u = R * u;
    y = u;
end

function y = CreateCircleGraph(A)
N = size(A,1);
L = A;
    for i= 1:N
         for j= 1:N  
             if j ~=  (i + 1)
                L(i,j) = 0;
             end
             if j == (i + 1) && L(i,j) == 1
                 L(i,j) = 1;
             end
             if i == N
                 L(i,1) = 1;
             end
         end
    end
y = L;
end

function y = Huddle(size, xuni, players, flagpos)
% Get new data and initialize new null velocities  

    teamSize = players(2) - players(1) + 1;
    x = xuni(1:2,players(1):players(2));                               % Extract single integrator states
    circularTargets = [flagpos(1) + size*cos( 0:2*pi/teamSize:2*pi*(1- 1/teamSize) ) ; flagpos(2) + size*sin( 0:2*pi/teamSize:2*pi*(1- 1/teamSize) ) ];
    errorToInitialPos = x - circularTargets;                      % Error
    y = -0.3.*errorToInitialPos;
end

function [x,y] = UpdateCircle(x, th, range)
    xunit = range * cos(th) + x(1,10);
    yunit = range * sin(th) + x(2,10);
    x = xunit;
    y = yunit;
end

function y = UpdateConnections(A, viewingDistance, x, N)
    for i= 1:N
         for j= 1:N  
             if j ~= i
                if (sqrt([1,1]*(x(:,j)-x(:,i)).^2)) < viewingDistance
                    A(i,j) = 1;
                else
                    A(i,j) = 0;
                end
             end
         end
    end
    y = A;
end

function y = GroupUp(distance, xuni, N, A)
% Get new data and initialize new null velocities  
    x = xuni(1:2,:);                             % Extract single integrator states
    u = zeros(2,N);                                % Initialize velocities to zero
     for i= 1:N
         for j= 1:N  
             if j ~= i
                u(:,i) = u(:,i) + (1 - (distance*A(i,j)) / (sqrt([1,1] * (x(:,j)-x(:,i)).^2))) * A(i,j) * (x(:,j)-x(:,i));
             end
         end
     end
     y = u;
end

function y = PopulateHazards(number,sizes,arena)
obstacleBuffer(1:3,1:number) = 999;
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
                if  sqrt((obstacleBuffer(2,a) - loc(1,1)).^2 + (obstacleBuffer(3,a) - loc(2,1)).^2) > ((actualSize + obstacleBuffer(1,a)) + 0.01) 
                    valid = valid + 1;
                end
            end
        
        end    

        CreateObstacle(actualSize,loc);
        obstacleBuffer(1,k) = actualSize;
        obstacleBuffer(2:3,k) = loc;
    end
    y = obstacleBuffer;
end

function CreateFlag(size,location,color)
patch(location(1) + [-cos(pi/4)*size/2,cos(pi/4)*size/2,-cos(pi/4)*size/2],location(2) + [-sin(pi/4)*size,0,sin(pi/4)*size],color);
end

function CreateObstacle(size,location)
P = [size, 0; 0, size];
center = location;
t = linspace(0,2*pi,30);
patch(P(1,1)*cos(t) + center(1), P(2,2)*sin(t) + center(2), [0.32,0.32,0.32]); %create a filled polygon - in this case an oval
end

function h = circle(x,y,r)
%hold on

th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit);
h.XDataSource = 'xunit';
h.YDataSource = 'yunit';
%hold off
end