function [isObstacleDetected, obstacleRegion] = obstacleDetected(probLeft, probRight, probFront, threshold)
    isObstacleDetected = (probLeft > threshold) || (probRight > threshold) || (probFront > threshold);
    if probLeft > threshold
        obstacleRegion = 'left';
    elseif probRight > threshold
        obstacleRegion = 'right';
    elseif probFront > threshold
        obstacleRegion = 'front';
    else
        obstacleRegion = 'none';
    end
end
