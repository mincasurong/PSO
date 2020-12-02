clc; close all; clear all;

global Max_iterations inertia swarm_size Return Para
graphON.Conv = 1;
graphON.D2 = 1;
graphON.D3 = 0;
reset = 1; reset_count = 30; reset_fig = 1;
for Return = 1:1 % 1000 -> Converge
    %% Initialization
    Max_iterations = 300; % Maximum number of iterations
    
    % Hyper-parameter
    % Hyper-parameter
    Para.max = 0.8; Para.min = 1;
    Para.C1i = Para.max;     Para.C2i = Para.min;     Para.iwi = Para.max;
    Para.C1f = Para.min;     Para.C2f = Para.max;     Para.iwf = Para.min;

    swarm_size = 20; % Number of particles
    LB = [-50 -50]; UB = [50   50]; % Boundaries
    xrange = UB(1) - LB(1); yrange = UB(2) - LB(2);
    
    swarm = zeros(swarm_size,4,2);
    Current.x = zeros(1,1); Current.y = zeros(1,1); Current.vx = zeros(1,1); Current.vy = zeros(1,1);
    Current.best_x = 0; Current.best_y = 0; Current.best_F = 0; Fval = zeros(1,1);
    Convergence.k = 0; Convergence.Curve = 0;
    
    % Initial Positions
    swarm(:,1,1) = rand(1,swarm_size) * xrange + LB(1);  % Swarm(Size, Actor, x-y)
    swarm(:,1,2) = rand(1,swarm_size) * yrange + LB(2);
    
    % Initial best value so far
    swarm(:,4,1) = 10;
    
    % Initial velocity
    swarm(:,2,:) = 0;
    
    %% Loop
    iter = 1;
    while(1)
        if iter == reset_count*reset
            swarm(:,1,1) = rand(1,swarm_size) * xrange + LB(1);  % Swarm(Size, Actor, x-y)
            swarm(:,1,2) = rand(1,swarm_size) * yrange + LB(2);
            swarm(:,4,1) = 10; swarm(:,2,:) = 0;
            reset = reset+1;
        end
        % Calculating fitness value for all particles
        for k=1:swarm_size
            swarm(k,1,1) = swarm(k,1,1) + swarm(k,2,1)/1; % updata x
            swarm(k,1,2) = swarm(k,1,2) + swarm(k,2,2)/1; % updata y
            if swarm(k,1,1) < LB(1), swarm(k,1,1) = LB(1);
            elseif swarm(k,1,1) > UB(1), swarm(k,1,1) = UB(1);
            elseif swarm(k,1,2) < LB(2), swarm(k,1,2) = LB(2);
            elseif swarm(k,1,2) > UB(2), swarm(k,1,2) = UB(2);
            end
            
            % The fitness function (DeJong) F(x,y) = x2 + y^2
            Fval(iter,k) = cost_function(swarm(k,1,1),swarm(k,1,2));
            
            if Fval(iter,k) < swarm(k,4,1)
                swarm(k,3,1) = swarm(k,1,1);  % Current x
                swarm(k,3,2) = swarm(k,1,2);  % Current y
                swarm(k,4,1) = Fval(iter,k);  % Currently the best fitness value
            end
        end
        
        % Search for the global best solution
        [temp, gbest] = min(swarm(:,4,1)); % global best position
        
        % Updating velocity vectors

        Para.iw = (Para.iwf-Para.iwi)*(Max_iterations-iter)/Max_iterations + Para.iwi;
        Para.C1 = (Para.C1f-Para.C1i)*(iter)/Max_iterations + Para.C1i;
        Para.C2 = (Para.C2f-Para.C2i)*(iter)/Max_iterations + Para.C2i;
        for k=1:swarm_size
            for h=1:2
                swarm(k,2,h) = rand*Para.iw*swarm(k,2,h) + ...
                    Para.C1*rand*(swarm(k,3,h) - swarm(k,1,h)) + ...
                    Para.C2*rand*(swarm(gbest,3,h)-swarm(k,1,h)); % PSO
            end
            
            % Output
            Current.x(iter,k) = swarm(k,1,1);        Current.y(iter,k) = swarm(k,1,2);
            if iter>2,
                Current.vx(iter,k) = swarm(k,2,1)/2;        Current.vy(iter,k) = swarm(k,2,2)/2;
            end
            Current.best_x(iter) = swarm(gbest,3,1); Current.best_y(iter) = swarm(gbest,3,2);
            Current.best_F(iter) = Fval(iter,gbest);
        end
        
        
        % Store the best fitness value in the convergence curve
        Convergence.Curve(iter) = swarm(gbest,4,1);
        if iter >= Max_iterations, break; end
        iter = iter + 1;
    end
    
    x = linspace(LB(1)-10, UB(1)+10); y = linspace(LB(2)-10, UB(2)+10);
    [X,Y] = meshgrid(x,y);   Z = cost_function(X,Y);
    
    %% Figure
    % Plot convergence cureve
    if graphON.Conv == 1
        figure('color','w')
        plot(Convergence.Curve,'b','linewidth',2);
        title('Convergence Curve');
        xlabel('Iterations')
        ylabel('Fitness Value')
        drawnow;
    end
    
    if graphON.D2 == 1
        % 2D
        reset_fig = 1;
        figure('color','w')
        for iter=1:Max_iterations
            contour(X,Y,Z); hold on; colormap; hold on
            for k=1:swarm_size
                plot(Current.x(iter,k),Current.y(iter,k),'mx','markersize',10,'linewidth',2); hold on;
                %                 plot([Current.x(iter,k)],[Current.y(iter,k)],'mx','markersize',10,'linewidth',2); hold on;
%                 if iter>2,
%                     quiver(Current.x(iter,k),Current.y(iter,k),...
%                         Current.vx(iter,k)-Current.vx(iter-1,k),...
%                         Current.vy(iter,k)-Current.vy(iter-1,k),0,'color','k','linewidth',2,...
%                         'MaxHeadSize',0.2);
%                 end
            end
            plot(Current.best_x(iter),Current.best_y(iter),'rs','markersize',20,'linewidth',2); hold on;
            
            if iter == reset_count*reset_fig, reset_fig = reset_fig + 1; end
            title(sprintf('Iteration : %d, reset: %.0f',iter,reset_fig-1))
            axis([-60 60 -60 60]); grid on
            drawnow;
            hold off;
        end
    end
    
    if graphON.D3 == 1
        % 3D
        reset_fig = 1;
        figure('color','w')
        for iter=1:Max_iterations
            meshc(X,Y,Z); hold on;
            colormap; hold on
            for k=1:swarm_size
                plot3(Current.x(iter,k),Current.y(iter,k),Fval(iter,k),'mx','markersize',10,'linewidth',2); hold on;
            end
            plot3(Current.best_x(iter),Current.best_y(iter),Current.best_F(iter),'rs','markersize',20,'linewidth',2); hold on;
            
            if iter == reset_count*reset_fig, reset_fig = reset_fig + 1; end
            title(sprintf('Iteration : %d, reset: %.0f',iter,reset_fig-1))
            axis([-60 60 -60 60 -30 30]); grid on
            drawnow;
            hold off;
            
        end
        xlabel('x'); ylabel('y')
    end
    
    for iter = 1:Max_iterations
        if Convergence.k == 0
            if iter > 5
                if abs(Convergence.Curve(iter)-Convergence.Curve(iter-1)) < 0.0001,
                    if abs(Fval(iter) - min(Convergence.Curve)) < 0.0001
                        Convergence.k = iter;
                    end
                end
            end
        end
    end
    if Convergence.k == 0, Convergence.k = Max_iterations; end
    Convergence.ksave(Return,1) = Convergence.k;
end

disp(sprintf('Convergence iterations: %.0f (%.2f)', mean(Convergence.ksave(:,1)), std(Convergence.ksave(:,1))))

%% Function
function [Fval] = cost_function(xvalue,yvalue)

Fval= 10*sin(3*pi*0.01*xvalue-10) + ...
    3.5*cos(3*pi*0.003*xvalue) + ...
    7.7*cos(3*pi*0.004*xvalue) + ...
    5*sin(3*pi*0.007*yvalue) + ...
    2.3*cos(3*pi*0.002*xvalue);

% Fval= 10*sin(3*pi*0.01*xvalue-10) + ...
%     3.5*cos(3*pi*0.013*xvalue) + ...
%     7.7*cos(3*pi*0.064*xvalue) + ...
%     5*sin(3*pi*0.007*yvalue) + ...
%     1.5*sin(3*pi*0.107*yvalue) + ...
%     2.5*sin(3*pi*0.307*yvalue) + ...
%     2.3*cos(3*pi*0.072*xvalue);
end