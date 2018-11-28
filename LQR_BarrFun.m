close all; clear all; clc

% Periodic System with LQR
dt = 0.005;
max_iter = 800;
trajectory = 100.*ones(2,max_iter);         % Autonomous trajectory
trajectory_ctr = 100.*ones(2,max_iter);     % Controlled trajectory
u_tr = zeros(2,max_iter);
opts = optimoptions(@quadprog,'Display','off');

A = [0.5,-0.4; 3, 0.5];                  

B = eye(2);
R = diag([0.3,0.1]);
M = diag([0.1,0.1]);

P = care(A,B,M,R);
Opt = R\B'*P;

% Initial conditions
x = [1;1];      % Autonomous state

% Plot stuff
figure, hold on, grid on
plot(0,0,'s','markersize',10,'markeredgecolor','none','markerfacecolor',[0.84,0.12,0.1])
autPt = plot(x(1), x(2), 'o','markersize',12,'markeredgecolor','none','markerfacecolor',[0.14,0.62,0.3]);
traPt = plot(trajectory(1,:),trajectory(2,:),'.','color',[0.14,0.62,0.3]);

% Ellipsoidal obstacle
obstacle = 1;
if obstacle
    P = [1/0.12, 0; 0, 1/0.09];
    center = [-0.1;0.7];
    t = linspace(0,2*pi,30);
    patch(1/sqrt( P(1,1))*cos(t) + center(1), 1/sqrt(P(2,2))*sin(t) + center(2),[0.32,0.32,0.32]);
end

axis(1.5.*[-1 1 -1 1])


for k = 1:max_iter
    
    % Optimal control input
    u_hat = -Opt*x;
    
    
    if obstacle
        % Barrier function
        h = (x - center )'*P*(x - center ) - 1;
        Abf = -2*(x - center)'*P;
        Bbf = 10*h^3;
        H = 2*eye(2);
        f = -2*u_hat;
        u = quadprog(sparse(H), double(f), Abf, Bbf, [],[], [], [], [], opts);
        u_tr(:,k) = u;
    else
        u = u_hat;
        u_tr(:,k) = u;
    end
    x = x + dt.*( A*x + B*u + R*randn(2,1));          
    
    
    % Plot Stuff
    trajectory(:,k) = x;
    set(autPt , 'xdata', x(1) ,'ydata', x(2))
    set(traPt , 'xdata', trajectory(1,:) ,'ydata', trajectory(2,:))
    drawnow
    pause(0.001)
    
end

