function [wL, wR] = avoidObstacle(probLeft, probRight, probFront)
        % Determine direction based on regional probabilities
        if (probLeft < probRight) && (probLeft < probFront)
            
            % min probability of sensors to get the angle approach
            %sensorAngle = min([prob(6), prob(7), prob(8)]);
            % Clearer path on the left
            % Adjust motor speeds to turn left
            wL = 0.15;  % Lower speed for left motor
            wR = 0.45;  % Higher speed for right motor
        elseif (probRight < probLeft) && (probRight < probFront)
            % Adjust motor speeds to turn right
            wL = 0.45;  % Higher speed for left motor
            wR = 0.15;  % Lower speed for right motor
%         elseif (probFront <= 0.15) && (probLeft <= 0.99) && (probLeft <= 0.99)
%             
%             wL = -0.3;
%             wR = 0.3;
%             pause(1);
                    
            
        else

                % Move forward if the front path is clear enough
                wL = 2;
                wR = 2;

        end
end
