function output = calculatePID(currentError, previousError, integralError, Kp, Ki, Kd)
   

    % Calculate the Proportional term
    P = Kp * currentError;
    
    % Update the Integral term
    % Note: You might want to implement windup guarding for the integral term
    
    I = Ki * integralError;
    
    % Calculate the Derivative term
    D = Kd * (currentError - previousError);
    
    % Calculate the total output
    output = P + I + D;
end
