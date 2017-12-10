% Calculate a path from qStart to xGoal
% input: qStart -> 1x4 joint vector describing starting configuration of
%                   arm
%        xGoal -> 3x1 position describing desired position of end effector
%        centers -> 3x3 positions of center of sphere
%        radii -> 1x3 vector of sphere radii
% output -> qMilestones -> nx4 vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You may output any number of
%                    milestones. The first milestone should be qStart. The
%                    last milestone should place the end effector at xGoal.
function qMilestones = Q2_RRT(rob,centers,radii,qStart,xGoal)
    qGoal = rob.ikine(transl(xGoal), zeros(1,4), [1,1,1,0,0,0]);
    
    nodes = qStart;
    edges = [];

    for i = 1:3000
        if ~rem(i, 100)
            fprintf('%s ', "*");
        end
        
        if rand < 0.1
            qRand = qGoal;
        else
            qRand = (rand(1,4)*(2*pi) - pi);
        end
        
        closestNeighbor = getClosestNeighbor(nodes, qRand);
        if ~Q1(rob, closestNeighbor, qRand, centers, radii)
            nodes(end+1, 1:4) = qRand;
            nodesHeight = size(nodes, 1);
            edges(end+1, 1:2) = [closestNeighbor, nodesHeight];
        end
                
        if(nodes(end, 1:4) == qGoal)
            fprintf('\nFound path after %d iterations', i);
            qMilestones = getMilestones(nodes, edges);
            return;
        end
    end
    fprintf('\nDid not find path after %d iterations. Retrying...\n', i);
    qMilestones = Q2_RRT(rob,centers,radii,qStart,xGoal);
end

function closestNeighbor = getClosestNeighbor(nodes, qRand)
    closestNeighbor = 1;
    nodesHeight = size(nodes, 1);
    for n = 2:nodesHeight
        possibleRow = nodes(n, 1:4);
        closestRow = nodes(closestNeighbor, 1:4);
        distPossible = sqrt(sum(qRand - possibleRow).^2);
        distClosest = sqrt(sum(qRand - closestRow).^2);
        if distPossible < distClosest
            closestNeighbor = n;
        end
    end
end

function qMilestones = getMilestones(nodes, edges)
    edgesMilestones = edges(end, 1:2);
    qMilestones = [];
    
    edgesHeight = size(edges, 1);
    for i = edgesHeight-1:-1:1
        if edges(i, 2) == edgesMilestones(end, 1)
            edgesMilestones(end+1, 1:2) = edges(i, 1:2);
        end
    end
    
    edgesMilestonesHeight = size(edgesMilestones, 1);
    for i = edgesMilestonesHeight:-1:1
        qMilestones(end+1, 1:4) = nodes(edgesMilestones(i,1), 1:4);
    end
    qMilestones(end+1, 1:4) = nodes(end, 1:4);
    
    fprintf('\nqMilestones:\n');
    disp(qMilestones);
end
