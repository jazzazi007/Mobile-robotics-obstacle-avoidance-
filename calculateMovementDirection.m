function [movementDirection] = calculateMovementDirection(robotPos, targetPos, sensorReadings, sensorAngles, K_att, K_rep, influenceRange)
    % Calculate Attractive Force
    attractiveForce = calculateAttractiveForce(robotPos, targetPos, K_att);

    % Initialize Net Repulsive Force
    netRepulsiveForce = [0, 0];

    % Calculate and Sum Repulsive Forces
    for i = 1:length(sensorReadings)
        distanceToObstacle = sensorReadings(i);
        repulsiveForce = calculateRepulsiveForce(distanceToObstacle, K_rep, influenceRange);
        angleRad = deg2rad(sensorAngles(i));
        netRepulsiveForce = netRepulsiveForce + [repulsiveForce * cos(angleRad), repulsiveForce * sin(angleRad)];
    end

    % Combine Attractive and Repulsive Forces
    %disp(size(attractiveForce));
    %disp(size(netRepulsiveForce));
    totalForce = attractiveForce + netRepulsiveForce;

    % Determine Movement Direction
    movementDirection = atan2(totalForce(2), totalForce(1));
     movementDirection = rad2deg(movementDirection);
end
