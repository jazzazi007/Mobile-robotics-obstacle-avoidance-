vrep=remApi('remoteApi');
vrep.simxFinish(-1);

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected')
    %Handle
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking); %Code to Access Motors
    
    [returnCode,sensor1]=vrep.simxGetObjectHandle(clientID,'Proximity_sensor1',vrep.simx_opmode_blocking);  %Code to Access Sensors
    [returnCode,sensor2]=vrep.simxGetObjectHandle(clientID,'Proximity_sensor2',vrep.simx_opmode_blocking);  %Code to Access Sensors
    [returnCode,sensor3]=vrep.simxGetObjectHandle(clientID,'Proximity_sensor3',vrep.simx_opmode_blocking);  %Code to Access Sensors
    [returnCode,sensor4]=vrep.simxGetObjectHandle(clientID,'Proximity_sensor4',vrep.simx_opmode_blocking);  %Code to Access Sensors    
    [returnCode,sensor5]=vrep.simxGetObjectHandle(clientID,'Proximity_sensor5',vrep.simx_opmode_blocking);  %Code to Access Sensors
    [returnCode,sensor6]=vrep.simxGetObjectHandle(clientID,'Proximity_sensor6',vrep.simx_opmode_blocking);  %Code to Access Sensors    
    [returnCode,sensor7]=vrep.simxGetObjectHandle(clientID,'Proximity_sensor7',vrep.simx_opmode_blocking);  %Code to Access Sensors
    [returnCode,sensor8]=vrep.simxGetObjectHandle(clientID,'Proximity_sensor8',vrep.simx_opmode_blocking);  %Code to Access Sensors   
    
    [returnCode,robot]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    
    [returnCode,endpos]=vrep.simxGetObjectHandle(clientID,'fire',vrep.simx_opmode_blocking);
    [returnCode,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
    
    sensors = [sensor1, sensor2, sensor3, sensor4, sensor5, sensor6, sensor7, sensor8];
    
    KpAng = 0.1; % Proportional gain
    KiAng = 0.000; % Integral gain
    KdAng = 0; % Derivative gain

    % Initialize variables
    currentErrorAng = 0;
    integralErrorAng = 0;
    angleThreshold = 0.01; % Threshold to determine when the target is reached
    
    KpPos = 1.1; % Proportional gain
    KiPos = 0; % Integral gain
    KdPos = 0; % Derivative gain

    % Initialize variables
    currentErrorPos = 0;
    integralErrorPos = 0;

    %Other Code      
    for n=1:8
        [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,sensors(n),vrep.simx_opmode_streaming);
            
    end

    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0.1,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0.1,vrep.simx_opmode_blocking);
    
    %[number returnCode,array position]=simxGetObjectPosition(number clientID,
     %number objectHandle,number relativeToObjectHandle,number operationMode)
    [returnCode,robotposition]=vrep.simxGetObjectPosition(clientID,robot, -1, vrep.simx_opmode_streaming);
    [returnCode,robotAngles]=vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming);
    [returnCode,endposition]=vrep.simxGetObjectPosition(clientID,endpos, -1, vrep.simx_opmode_streaming);
    [returnCode,track]=vrep.simxGetObjectPosition(clientID,human, -1, vrep.simx_opmode_streaming);

    [returnCode,resolution,image]=vrep.simxGetVisionSensorDepthBuffer2(clientID,camera,vrep.simx_opmode_streaming);

    tic
    for i=1:1000
        [returnCode,robotposition]=vrep.simxGetObjectPosition(clientID,robot, -1, vrep.simx_opmode_buffer);
        [returnCode,endposition]=vrep.simxGetObjectPosition(clientID,endpos, -1, vrep.simx_opmode_buffer);
        [returnCode,robotAngles]=vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_buffer);
        
        
        [returnCode,resolution,image]=vrep.simxGetVisionSensorDepthBuffer2(clientID,camera,vrep.simx_opmode_buffer);
        
        %%%%desired distance and angle
        [distance, angle, angleNorm] = computeDistanceAndAngle(robotposition, endposition, robotAngles); %distance m, angle deg
        
        %%%%%PID controller Angular
        previousErrorAng = currentErrorAng;
        
        currentErrorAng = angle;
        
        integralErrorAng = integralErrorAng + currentErrorAng;
        
        outputAng = calculatePID(currentErrorAng, previousErrorAng, integralErrorAng, KpAng, KiAng, KdAng);
        
        %%%%%PID controller position
        previousErrorPos = currentErrorPos;
        
        currentErrorPos = distance;
        
        integralErrorPos = integralErrorPos + currentErrorPos;
        
        outputPos = calculatePID(currentErrorPos, previousErrorPos, integralErrorPos, KpPos, KiPos, KdPos);


        thirdElements = zeros(1, 8);  % Array to store third elements
        prob = zeros(1,8);
        %%%%%%%%%%%%%%%
        % Define sensor angles (example angles)
        sensorAngles = linspace(0, 180, 8); % degrees

        % Initialize net force vector
        netForceX = 0;
        netForceY = 0;
        
        influenceRange = 1;
        K_rep = 1;
        K_att = 1;



        for n=1:8
            [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,sensors(n),vrep.simx_opmode_buffer);
            if detectionState == 0
                detectedPoint = [0, 0, 0];  % Set to default if no detection
            end
            thirdElements(n) = detectedPoint(3);  % Storing the third element
            
            if thirdElements(n)<=0.001
                prob(n) = thirdElements(n);
            else
                prob(n) = (abs(thirdElements(n)-1.5))/1.5;
            end
            
            
            
        end
        sensorReadings = thirdElements;
        [movementDirection] = calculateMovementDirection(robotposition, endposition, sensorReadings, sensorAngles, K_att, K_rep, influenceRange);
        % Define the sensor indices for each region
        probLeft = (prob(6)+prob(7)+prob(8))/3
        probRight = (prob(1)+prob(2)+prob(3))/3
        probFront = (prob(5)+prob(4))/2

%%%%%%%%%%%%%%%%%%%%%
        % Determine direction based on regional probabilities
%       if (((probLeft > 0) && (probRight > 0)) || ((probLeft > 0) && (probFront > 0)) || (probRight > 0) && (probFront > 0))
%         [wL, wR] = avoidObstacle(probLeft, probRight, probFront);
%       else
      if (angle>30)
          if ((probLeft>0.15) || ((probLeft > 0.15) && (probRight > 0.15)) || ((probLeft > 0.15) && (probFront > 0.15)) || (probRight > 0.15) && (probFront > 0.15))
              [wL, wR] = avoidObstacle(probLeft, probRight, probFront);
          else
           maxSpeed = 2;
           wL = max(-maxSpeed, min(maxSpeed, (outputPos - outputAng)));
           wR = max(-maxSpeed, min(maxSpeed, (outputPos + outputAng)));
          end 
      elseif (angle<-30)
          if ((probRight>0.15) || ((probLeft > 0.15) && (probRight > 0.15)) || ((probLeft > 0.15) && (probFront > 0.15)) || (probRight > 0.15) && (probFront > 0.15))
            [wL, wR] = avoidObstacle(probLeft, probRight, probFront);
          else 
           maxSpeed = 2;
           wL = max(-maxSpeed, min(maxSpeed, (outputPos - outputAng)));
           wR = max(-maxSpeed, min(maxSpeed, (outputPos + outputAng)));
          end
      elseif (angle <=30 && angle >=-30)
          if ((probFront>0.15) || ((probLeft > 0.15) && (probRight > 0.15)) || ((probLeft > 0.15) && (probFront > 0.15)) || (probRight > 0.15) && (probFront > 0.15))
              [wL, wR] = avoidObstacle(probLeft, probRight, probFront);
          else
              maxSpeed = 2;
           wL = max(-maxSpeed, min(maxSpeed, (outputPos - outputAng)));
           wR = max(-maxSpeed, min(maxSpeed, (outputPos + outputAng)));
          end
      end

              
          
     
%%%%%%%%%%%%%%%%%%%%%%%%%
    

       %    maxSpeed = 1;
        %   wL = max(-maxSpeed, min(maxSpeed, (outputPos - outputAng)));
         %  wR = max(-maxSpeed, min(maxSpeed, (outputPos + outputAng)));

        % Set motor speeds
        [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_Motor, wL, vrep.simx_opmode_blocking);
        [returnCode] = vrep.simxSetJointTargetVelocity(clientID, right_Motor, wR, vrep.simx_opmode_blocking);

        
       
       %imshow(image);
       %disp(thirdElements);

       pause(0.1);
    end
    
    %[returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);

    vrep.simxFinish(-1);
end

vrep.delete();
    