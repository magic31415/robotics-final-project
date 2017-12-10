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
function qMilestones = Q2_PRM(rob, centers, radii, qStart, xGoal)
    numNodes = 200;
    qGoal = rob.ikine(transl(xGoal), zeros(1,4), [1,1,1,0,0,0]);
    nodes = [];
    edges = [];
    
    startTime = posixtime(datetime(clock));
    
    % Get random nodes and corresponding edges
    for i = 1:numNodes
        if ~rem(i, 10)
            fprintf('%s ', "*");
        end
        
        qRand = rand(1,4)*(2*pi) - pi;
        if ~robotCollisionHelper(rob, qRand, centers, radii)
            nodes = [nodes; qRand];
            edges = [edges;...
                     createEdges(qRand,...
                                 findClosestNodes(qRand, nodes(1:end-1, 1:4), rob, centers, radii),...
                                 nodes)];
        end
    end
    
    fprintf('\nCreated %d nodes\n', size(nodes, 1));
    fprintf('\nCreated %d edges\n', size(edges, 1));
    elapsedTime = round(posixtime(datetime(clock)) - startTime);
    fprintf('\nTook about %d seconds\n', elapsedTime);
        
    qMilestones = milestonesFinder(qStart, qGoal, nodes, edges, rob, centers, radii);
end

% checks if configuration is in collision with any of the obstacles
function bool = robotCollisionHelper(rob, q, centers, radii)
    for i = 1:size(radii,2)
        if robotCollision(rob, q, centers(1:3,i), radii(i))
            bool = 1;
            return
        end
    end
    bool = 0;
end

% connects a node to close nodes
function createdEdges = createEdges(q, closeNodes, nodes)
    createdEdges = [];

    for i = 1:size(closeNodes)
        qClose = closeNodes(i,1:4);
        createdEdges = [createdEdges; indexOf(q, nodes) indexOf(qClose, nodes)];
    end
end

% finds the closest other nodes to the given node, if a clear path exists
function closeNodes = findClosestNodes(q, nodes, rob, centers, radii)
    closeNodes = [];
    thresholdDistance = 0.5;
    
    for i = 1:size(nodes,1)
        qTest = nodes(i,1:4);
        
        if distanceBetween(q, qTest) < thresholdDistance && ~Q1(rob, q, qTest, centers, radii)
            closeNodes = [closeNodes; qTest];
        end
    end
end

% finds the closest node to a given node, where there is a clear path
% used for connecting the start and goal configurations to the path
function closestNode = getClosestTo(q, nodes, rob, centers, radii)
    closestNode = [];
    shortestDistance = inf;
    
    for i = 1:size(nodes,1)
        qTest = nodes(i,1:4);
        distance = distanceBetween(q, qTest);
        
        if (distance < shortestDistance) && ~Q1(rob, q, qTest, centers, radii)
            closestNode = qTest;
            shortestDistance = distance;
        end
    end
end

% builds a path of milestones from start to goal using dijkstra's algorithm
function path = milestonesFinder(q1, q2, nodes, edges, rob, centers, radii)

    close1 = getClosestTo(q1, nodes, rob, centers, radii);
    close2 = getClosestTo(q2, nodes, rob, centers, radii);
    
    path = [];
    for index = findDijkstrasPath(close1, close2, nodes, edges)
        path = [path; nodes(index,1:4)];
    end
    path = [q1; path; q2];
    
    fprintf('\nqMilestones:\n');
    disp(path);
end


% DIJKSTRAS =============================================================
% Algorithm from: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
% Finds the shortest path between two nodes
function dijkstrasPath = findDijkstrasPath(q1, q2, nodes, edges)
    unvisited = 1:size(nodes,1);
    distances = inf(1, size(unvisited, 2));
    prevs = zeros(1, size(unvisited, 2));
    
    startIndex = indexOf(q1, nodes);
    goalIndex = indexOf(q2, nodes);
    distances(startIndex) = 0;
    dijkstrasPath = [];
    
    iterations = 0;
    
    while(size(unvisited, 2) > 0)
        iterations = iterations + 1;
        
        minNode = getMinNode(unvisited, distances);
        unvisited = remove(minNode, unvisited);
        
        % disp([minNode, goalIndex, iteration])
        if minNode == goalIndex
            fprintf('\nFound path after %d iterations', iterations)
            
            while prevs(minNode) > 0
                dijkstrasPath = [minNode dijkstrasPath];
                minNode = prevs(minNode);
            end
            
            dijkstrasPath = [startIndex dijkstrasPath];
            break;
        else
            neighbors = getNeighborsOf(minNode, edges, unvisited);

            for neigh = neighbors
                dist = distances(minNode) + distanceBetween(nodes(minNode, 1:4), nodes(neigh, 1:4));

                if dist < distances(neigh)
                    distances(neigh) = dist;
                    prevs(neigh) = minNode;
                end
            end
        end
    end
end

% Finds unvisited nodes that share edges with the given node
function neighbors = getNeighborsOf(node, edges, unvisited)
    neighbors = [];
    
    for i = 1:size(edges, 1)
        frontNode = edges(i, 1);
        backNode = edges(i, 2);
        if (frontNode == node) && (ismember(backNode, unvisited))
            neighbors = [neighbors backNode];
        elseif (backNode == node) && (ismember(frontNode, unvisited))
            neighbors = [neighbors frontNode];
        end
    end
end

% Finds the unvisited node with the shortest distance
function minNode = getMinNode(unvisited, distances)
    minDistance = inf;
    minNode = 0;
    
    for node = unvisited
        testDistance = distances(node);
        
        if testDistance < minDistance
            minDistance = testDistance;
            minNode = node;
        end
    end
    
    % if all unvisited nodes have distance infinity
    % they are unreachable from the starting node
    if ~minNode
        error('Error: Path not found.');
    end
end

% get index of a row in a 2D array
function i = indexOf(row, array)
    for i = 1:size(array,1)
        if row == array(i, 1:end)
            break;
        end
    end
end

% get index of an item in a 1D list
function i = listIndexOf(item, list)
    for i = 1:size(list,2)
        if item == list(i)
            break;
        end
    end
end

% remove an item from a 1D list
function newList = remove(item, list)
    list(listIndexOf(item, list)) = [];
    newList = list;
end

% calculates the distance between two nodes (used by PRM as well)
function distance = distanceBetween(q1, q2)
    distance = sqrt(sum(q1 - q2).^2);
end
