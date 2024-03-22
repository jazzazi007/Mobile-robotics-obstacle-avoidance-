function [distance, angle, angleNorm] = computeDistanceAndAngle(robotPosition, targetPosition, robotOrientation)
    % Calculate the distance between the robot and the target
    distance = sqrt((targetPosition(1) - robotPosition(1))^2 + (targetPosition(2) - robotPosition(2))^2);
    % Calculate the angle from the robot to the target
    % The angle to the target is the angle of the line connecting the robot and the target, relative to the global frame
    angleToTargetGlobalFrame = atan2(targetPosition(2) - robotPosition(2), targetPosition(1) - robotPosition(1));
    % The orientation of the robot is its angle relative to the global frame
    % The desired angle of motion relative to the robot's frame is the angle to the target minus the robot's orientation
    robotOrientation3 = robotOrientation(3);
    angleDiff = mod(rad2deg(angleToTargetGlobalFrame) - rad2deg(robotOrientation3) + 180, 360) - 180;
   % angle = angleToTargetGlobalFrame - robotOrientation3;  % Assuming the robot's orientation in the 3rd component is the heading angle
    %angle = rad2deg(angle)
    angle = angleDiff;
    % Normalize the angle to the range [-pi, pi]
    angleNorm = atan2(sin(angle), cos(angle));
    angleNorm = rad2deg(angleNorm);
end
