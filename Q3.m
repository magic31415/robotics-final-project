% Smooth path given in qMilestones
% input: qMilestones -> nx4 vector of n milestones. 
%        centers -> 3x3 positions of center of sphere
%        radii -> 1x3 vector of sphere radii
% output -> qMilestones -> mx4 vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You should output a number of
%                    milestones m<=n.
function qMilestonesSmoothed = Q3(rob, qMilestones, centers, radii)
    qMilestonesSmoothed = qMilestones(1, 1:4);
    startRow = 1;
    goalRow = size(qMilestones, 1);
    tryRow = goalRow;
    
    while startRow ~= goalRow
        % fprintf('%d %d \n', startRow, tryRow);
        qStart = qMilestones(startRow, 1:4);
        qTry = qMilestones(tryRow, 1:4);
        if ~Q1(rob, qStart, qTry, centers, radii)
            qMilestonesSmoothed(end+1, 1:4) = qTry;
            startRow = tryRow;
            tryRow = goalRow;
        else
            tryRow = tryRow - 1;
        end
    end
    fprintf('\nqMilestonesSmoothed:\n');
    disp(qMilestonesSmoothed);
end
