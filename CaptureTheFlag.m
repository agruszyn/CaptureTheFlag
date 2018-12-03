clear all, close all, clc

%(------------------------------------------ PARAMETERS ------------------------------------------)
%%
hazardTotal = 7;                            % Total number of obstacles
hazardSizes = [0.05,0.3];
hazardWindow = [-1.5,1.5;-1.7,1.7];

flagSize = 0.25;
redFlagpos = [2,1.1];
blueFlagpos = [-2,-1.1];

N = 10;                                     % Total number of players team
init(1,1:N) = 2.5;                          % starting X position
init(2,1:N) = linspace(-1.5,1.5,10);        % starting Y position
init(3,1:N) = 2;                            % starting rotation
viewingDistance = 1;                        % Range of vision for each player

max_iter = 2000;                            % max script time
dt = 0.01;                                  % numerical steplength

lambda = 0.05;                              % lambda for barrier certificate
safety = 1.5 * r.robot_diameter;            % safety distance for barrier certificate
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
    
    % group up together
    u = GroupUp(0.5, xuni, N, A);
    
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
patch(location(1) + [0,-cos(pi/4)*size,0],location(2) + [0,sin(pi/4)*size,sin(pi/2)*size],color);
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