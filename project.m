% From Rob Platt's HW2
%
% You must run startup_rvc FIRST before running this function.
% DO NOT MODIFY THIS FILE!
% input: questionNum -> Integer between 1 and 3 that denotes question
%                       number to run.
function project(questionNum)

%     close all;
    
    if nargin < 1
        error('Error: please enter a question number as a parameter');
%         questionNum = 2;
    end
    
    % set up robot and initial joint configuration
    rob = createRobot(); % 4 joints
    % mdl_puma560 % 6 joints
    
    qStartRob = [0 -0.78 0 -0.78];
    % qStartPuma = [0 -pi -pi/2 0 0 0];
    
	xGoal = [0.5; 0.0; -0.5];
    
    sphereCenter1 = [0.5; 0.0; 0.0];
    sphereCenter2 = [0.0; 0.5; 0.0];
    sphereCenter3 = [0.0; 0.0; 0.5];
    
    sphereCenters = [sphereCenter1 sphereCenter2 sphereCenter3];
    sameCenters = [sphereCenter1 sphereCenter1 sphereCenter1];
    
    sphereRadii = [0.2 0.3 0.1];
    sameRadii = [0.2 0.2 0.2];
        
    % plot robot
    rob.plot(qStartRob)
    % p560.plot(qStartPuma);
    
    % plot spheres
    for i = 1:size(sameRadii,2)
        hold on;	
        drawSphere(sphereCenters(1:3, i), sphereRadii(i));
    end

    % only using one obstacle
    if questionNum == 1
        collision = Q1(rob, qStartRob, [-0.9391, -2.5286, 1.4566, -0.3478], sameCenters, sameRadii);
        display(['this should be 1: ',int2str(collision)])
        collision = Q1(rob, qStartRob, [0 -0.78 0 -1.5], sameCenters, sameRadii);
        display(['this should be 0: ',int2str(collision)])        
        collision = Q1(rob, qStartRob, [0 -0.78 0 -0.1], sameCenters, sameRadii);
        display(['this should be 1: ',int2str(collision)])
        
    % only using one obstacle
    % not working
    elseif questionNum == 2.1
        qMilestones = Q2_RRT(rob,sameCenters,sameRadii,qStartRob,xGoal);
        plotterHelper(rob, qMilestones);

    elseif questionNum == 2.2
        qMilestones = Q2_PRM(rob,sphereCenters,sphereRadii,qStartRob,xGoal);
        plotterHelper(rob, qMilestones);
        
    % only using one obstacle
     % not working
    elseif questionNum == 3.1
        qMilestones = Q2_RRT(rob,sameCenters,sameRadii,qStartRob,xGoal);
        qMilestones = Q3(rob,qMilestones,sameCenters,sameRadii);
        plotterHelper(rob, qMilestones)
        
    elseif questionNum == 3.2
        qMilestones = Q2_PRM(rob,sphereCenters,sphereRadii,qStartRob,xGoal);
        qMilestones = Q3(rob,qMilestones,sphereCenters,sphereRadii);
        plotterHelper(rob, qMilestones)
        
    else
        error('Error: question number out of range.');
    end
    

end

function plotterHelper(rob, qMilestones)
    % interpolate and plot direct traj from start to goal
    qTraj = interpMilestones(qMilestones);
    rob.plot(qTraj);
end

function traj = interpMilestones(qMilestones)

    d = 0.05;
%     traj = qMilestones(1,:);
    traj = [];
    for i=2:size(qMilestones,1)        
        delta = qMilestones(i,:) - qMilestones(i-1,:);
        m = max(floor(norm(delta) / d),1);
        vec = linspace(0,1,m);
        leg = repmat(delta',1,m) .* repmat(vec,size(delta,2),1) + repmat(qMilestones(i-1,:)',1,m);
        traj = [traj;leg'];
        
    end
end

function qPath = getPath(tree)

    m = 10;
    idx = size(tree,1);
    path = tree(end,1:end-1);
    
    while(idx ~= 1)
        
        curr = tree(idx,1:end-1);
        idx = tree(idx,end);
        next = tree(idx,1:end-1);
        path = [path;[linspace(curr(1),next(1),m)' linspace(curr(2),next(2),m)' 
            linspace(curr(3),next(3),m)' linspace(curr(4),next(4),m)']];
        
    end
    qPath = path(end:-1:1,:);
    
end


function rob = createRobot()

    L(1) = Link([0 0 0 1.571]);
    L(2) = Link([0 0 0 -1.571]);
    L(3) = Link([0 0.4318 0 -1.571]);
    L(4) = Link([0 0 0.4318 1.571]);
    
    rob = SerialLink(L, 'name', 'robot');

end

function drawSphere(position,diameter)

    [X,Y,Z] = sphere;
    X=X*diameter;
    Y=Y*diameter;
    Z=Z*diameter;
    X=X+position(1);
    Y=Y+position(2);
    Z=Z+position(3);
    surf(X,Y,Z);
    %~ shading flat

end


