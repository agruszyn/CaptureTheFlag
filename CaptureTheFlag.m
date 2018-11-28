clear all, close all, clc

N=7;                        % Agents per team
Offense = 5;
Defense = 5;

dt=0.01;                   % numerical steplength
max_iter = 1000;
center = [0;0];
videoFLag = 0;                          % Change to 1 to record video
circularInitialConditions = 0;          % Change to 0 for random initial condition (needed for Q2.d)
                           

% Initialize robotarium
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

xuni = r.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,:);                                            % x-y positions only
r.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
r.step();                                                % Run robotarium step

[si_to_uni_dyn] = create_si_to_uni_mapping3();

% FILL THIS PART - Define formation weights    
W = 0.15*[-2 2 0 sqrt(37) 0 sqrt(8) 0; 0 -2 2 sqrt(17) sqrt(8) 0 0; 0 0 -2 sqrt(5) 2 0 0; 0 0 0 -2 0 0 0; 0 0 0 sqrt(5) -2 0 0; 0 0 sqrt(8) sqrt(17) 2 -2 0; 0 sqrt(8) 0 sqrt(37) 0 2 -2];

% Reach initial positions on a circle
if circularInitialConditions        
    circularTargets = [ cos( 0:2*pi/N:2*pi*(1- 1/N) ) ; sin( 0:2*pi/N:2*pi*(1- 1/N) ) ];
    errorToInitialPos = x - circularTargets;                % Error
    errorNorm = sqrt([1,1]*(errorToInitialPos.^2));               % Norm of error
    while max( errorNorm ) > 0.05
        % Update state variables        
        %xuni = rbtm.get_poses();                            % States of real unicycle robots
        xuni = r.get_poses();                            % States of real unicycle robots
        x = xuni(1:2,:);                                    % x-y positions
        
        % Update errors
        errorToInitialPos = x - circularTargets;
        errorNorm = sqrt([1,1]*(errorToInitialPos.^2));
        
        % Comput control inputs
        u = -0.3.*errorToInitialPos;
        dx = si_to_uni_dyn(u, xuni);
        r.set_velocities(1:N, dx);                       % Assign dummy zero velocity
        r.step();                                        % Run robotarium step
    end
    disp('Initial positions reached')
    
end

for k = 1:max_iter
    
    % Get new data and initialize new null velocities
    xuni = r.get_poses();                                % Get new robots' states
    x = xuni(1:2,:);                                        % Extract single integrator states

    u=zeros(2,N);                                           % Initialize velocities to zero
    
     % FILL THIS PART!!!
     for i= 1:N
         for j= 1:N     
             %u(:,i) = u(:,i) + L(i,j)*x(:,j);
             if j ~= i && W(i,j) ~= 0
             u(:,i) = u(:,i) + (1 - W(i,j) / (sqrt(  [1,1]*(x(:,j)-x(:,i)).^2  )))  *  (x(:,j)-x(:,i));
             
             if W(i,j) ~= 0
             
             end
             end
             if i == 4 && j == 1
             u(:,i) = u(:,i) + (center(:,j)-x(:,i));
             end
         end
     end
     disp(k);
    dx = si_to_uni_dyn(u, xuni);                            % Convert single integrator inputs into unicycle inputs
    r.set_velocities(1:N, dx); r.step();              % Set new velocities to robots and update

end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();
r.debug();
