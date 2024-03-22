function repulsiveForce = calculateRepulsiveForce(distanceToObstacle, K_REP, INFLUENCE_RANGE)


    if distanceToObstacle <= INFLUENCE_RANGE
        repulsiveForce = K_REP * ((1 / distanceToObstacle) - (1 / INFLUENCE_RANGE)) / (distanceToObstacle ^ 2);
    else
        repulsiveForce = 0;
    end
end
