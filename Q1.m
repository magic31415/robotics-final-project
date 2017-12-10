% This function takes two joint configurations and the parameters of the
% obstacle as input and calculates whether a collision free path exists
% between them.
% 
% input: q1, q2 -> start and end configuration, respectively. Both are 1x4
%                  vectors.
%        centers -> 3x3 positions of center of sphere
%        radii -> 1x3 vector of sphere radii
%        rob -> SerialLink class that implements the robot
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = Q1(rob, q1, q2, centers, radii)
    numSteps = 100;
    step_sizes = (q2 - q1)/numSteps;

    for n = 0:numSteps
        qTest = q1 + (n * step_sizes);
        for i = 1:size(centers, 2)
            if robotCollision(rob, qTest, centers(1:3, i), radii(i))
                collision = 1;
                return;
            end
        end
    end
    collision = 0;
end