function attractiveForce = calculateAttractiveForce(robotPos, targetPos, K_att)
    % Calculate the difference in position
    targetPos = [targetPos(1), targetPos(2)];
    robotPos = [robotPos(1), robotPos(2)];
    positionDifference = targetPos - robotPos;

    % Calculate the attractive force
    % The force magnitude is proportional to the distance to the target
    forceMagnitude = K_att * norm(positionDifference);

    % Normalize the position difference to get the direction
    direction = positionDifference / norm(positionDifference);

    % The attractive force vector
    attractiveForce = forceMagnitude * direction;
end
