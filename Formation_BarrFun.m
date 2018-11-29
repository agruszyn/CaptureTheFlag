clear all, close all, clc

N=3;                        % Number of agents
dt=0.03;                   % numerical steplength
max_iter = 1000;           
Delta = 1.2;
opts = optimoptions(@quadprog,'Display','off');


% Initialize robots
x = [-2.8 -2.2 -2.2; 0.2 0.5 -0.4];                                            % x-y positions only
figure(1), hold on

% Cyclic graph
A = diag(ones(N-1,1),1) + diag(ones(N-1,1),-1) ;
A(1,N) = 1; A(N,1) = 1;
L = diag(sum(A)) - A;
E = [1,2; 2,3; 3,1];

% Goal point
goal = [2.3;0.4];
plot(goal(1),goal(2),'*','markersize',12)

% Ellipsoidal obstacle
P = [0.5, 0; 0, 0.5];
center = [0;0];
t = linspace(0,2*pi,30);
patch( P(1,1)*cos(t) + center(1), P(2,2)*sin(t) + center(2),[0.32,0.32,0.32]); %create a filled polygon - in this case an oval
axis(3.*[-1,1,-1,1])            %define axis of the graph

% Plot
edgePlot = plot([x(1,E(:,1)),x(1,E(:,2))] , [x(2,E(:,1)),x(2,E(:,2))], 'LineWidth',2, 'Color',[0.3,0.3,0.3]); 
agenPlot = plot(x(1,:),x(2,:),'o','markersize',12,'MarkerFaceColor',[0.12,0.49,0.65],'MarkerEdgeColor','none');
  
tot_constraints = 6;                                            % 3 agent-to-obstacle constraints + 3 agent-to-agent constraints
for iter = 1:max_iter
    
    dx = zeros(2,N);                                           % Initialize velocities to zero         
    for i = 1:N                
        for k = find(A(:,i))'
            if ~isempty(k)
                dx(:,i) = dx(:,i) + (norm(x(:,i)-x(:,k)) - 1)*( x(:,k)-x(:,i) );
            end
        end
        dx(:,i) = dx(:,i) + 0.1.*(goal-x(:,i))./norm(goal-x(:,i));
    end
     
    
    % Inizialize zero matrices for storing constraints
    Abf = zeros(tot_constraints, 2*N);
    Bbf = zeros(tot_constraints,   1);
    count = 1;
    % Connectivity constraints
    for k = 1:3
        i = E(k,1);
        j = E(k,2);
        h_ij = (Delta)^2 - (x(:,i)-x(:,j))'*(x(:,i)-x(:,j));            
        Abf(count , 2*i-1:2*i ) =  2*( x(:,i)-x(:,j) );
        Abf(count , 2*j-1:2*j ) = -2*( x(:,i)-x(:,j) );   
        Bbf(count) = 1e5*h_ij^3;
        count = count+1;
    end
    
    % Obstacle avoidance
    for i = 1:3
        h_obs = (x(:,i) - center )'*P*(x(:,i) - center ) - 1;
        Abf(count, 2*i-1:2*i) = -2*( x(:,i) - center )'*P;
        Bbf(count) = 10*h_obs^3;
        count = count + 1;
    end

    % BF Solution
    H = 2*eye(2*N);
    f = -2*reshape(dx,[2*N,1]);
    u = quadprog(sparse(H), double(f), Abf, Bbf, [],[], [], [], [], opts);
    u = reshape(u,[2,N]);

    x = x + u*dt; 
    
    set(agenPlot,'xdata',x(1,:),'ydata',x(2,:))
    set(edgePlot,'Xdata',[x(1,E(:,1)),x(1,E(:,2))], 'Ydata',[x(2,E(:,1)),x(2,E(:,2))])
    drawnow
    disp(iter);
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
