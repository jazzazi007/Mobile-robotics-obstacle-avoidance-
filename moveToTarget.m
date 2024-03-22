function [wL, wR] = moveToTarget(outputDistance, outputOrientation)
    % Basic implementation: combine distance and orientation outputs to set wheel speeds
    % This could be more sophisticated based on your robot's kinematics
    wL = outputDistance - outputOrientation;
    wR = outputDistance + outputOrientation;
    
    % Ensure the wheel speeds are within the robot's limits
    maxSpeed = 3;
    wL = max(min(wL, maxSpeed), -maxSpeed);
    wR = max(min(wR, maxSpeed), -maxSpeed);
end
