function mouse(serPort)
    global F_Sonar; global F; global L; global R; A = .3; 
    wrongWall = true; direction = 0;
     
    while true
        updateSensors(serPort);  
        direction = direction + AngleSensorRoomba(serPort);
        disp(direction);
        if direction > 3  %+ means going clockwise - ergo...around outter wall!!!
            disp('going clockwise - must be outer wall');
            wrongWall = true;
            break;
        elseif direction < -3  %- means going anticlockwise - ergo..around home!
            disp('going anticlockwise - must be house');
            wrongWall = false;
            break;
        end
        
        if L < A || R > A
            SetDriveWheelsCreate(serPort, .1,.4) %spin clockwise  .1.4   
        elseif L > A || R < A
            SetDriveWheelsCreate(serPort, .4,.1) %spin anticlockwise .4.1    
        else 
            SetDriveWheelsCreate(serPort, .3,.3) %spin anticlockwise     
        end
    end
    disp('test complete'); disp(direction);
    
    %get away from this wall and onto the other one!
    if wrongWall == true
%         while F_Sonar < 1 
%             turnAngle(serPort,.1,1);
%         end
        turnAngle(serPort,.1,40);
        travelDist(serPort,1,2);
        while F_Sonar > 1 
            updateSensors(serPort);
            travelDist(serPort,1,.1);
        end
        attractiveWalls(serPort);
        mouse(serPort);
    end
    
    
    updateSensors(serPort); 
    while L > .5 
        updateSensors(serPort); 

        if L < A || R > A
            SetDriveWheelsCreate(serPort, .1,.4) %spin clockwise  .1.4   
        elseif L > A || R < A
            SetDriveWheelsCreate(serPort, .4,.1) %spin anticlockwise .4.1    
        else 
            SetDriveWheelsCreate(serPort, .3,.3) %spin anticlockwise     
        end
    end
    
    while F_Sonar > 1
        updateSensors(serPort);
        travelDist(serPort,.1*F_Sonar,.1);
    end
end

function updateSensors(serPort)
    %INIT VARIABLES
    global distBeacon;  global L; global R; global F; global Camera; global F_Sonar; global L_Sonar; global R_Sonar;
    R_Bump = false; L_Bump = false; F_Bump = false; L_Sonar = 100; R_Sonar = 100; F_Sonar = 100; L_Lidar = 100; R_Lidar = 100; F_Lidar = 100;    
    %INIT BUMP SENSORS
    [R_Bump, L_Bump, ~, ~, ~, F_Bump] = BumpsWheelDropsSensorsRoomba(serPort);   
    %INIT CAMERA SENSORS    
    Camera = CameraSensorCreate(serPort); [~, distBeacon, ~] = CameraSensorCreate(serPort);
    %INIT SONAR SENSORS    
    L_Sonar = ReadSonar(serPort, 3); F_Sonar = ReadSonar(serPort, 2); R_Sonar = ReadSonar(serPort, 1); 
    if ~any(F_Sonar) F_Sonar = 100;end
    if ~any(L_Sonar) L_Sonar = 100;end
    if ~any(R_Sonar) R_Sonar = 100;end
    %INIT LIDAR SENSORS    
    LIDAR = LidarSensorCreate(serPort); [L_Lidar,~] = min(LIDAR(511:681)); [R_Lidar,~] = min(LIDAR(1:170)); [F_Lidar,~] = min(LIDAR(255:425));  %LidarSensorCreate = 680-vector (Right: 0 to 341. front = 341.  left: 341 to 681)  
    if ~any(F_Lidar) F_Lidar = 100;end
    if ~any(L_Lidar) L_Lidar = 100;end
    if ~any(R_Lidar) R_Lidar = 100;end
    %SETUP  
    F = F_Lidar; if (F_Bump == true) F = -1; end  
    L = L_Lidar; if (L_Bump == true) L = -1; end 
    R = R_Lidar; if (R_Bump == true) R = -1; end 
end

function turnAccurate(serPort, desiredAngle)
    turnAngle(serPort, .1, desiredAngle); 
    actualAngle = AngleSensorRoomba(serPort); actualAngle = ( actualAngle*180 ) /pi; %rad -> deg
    if (abs(actualAngle - desiredAngle) > .5) turnAccurate(serPort, desiredAngle - actualAngle); end
end

function attractiveWalls(serPort)
    global F; global L; global R; 
    R_wheel = 0.2; L_wheel = 0.2;
    e = 0.1;
    
    updateSensors(serPort);
    while F > .1 && L > .3 && R > .3;
        SetDriveWheelsCreate(serPort, R_wheel, L_wheel)        
        updateSensors(serPort);
        R_wheel = R*e; L_wheel = L*e;
    end
    turnAccurate(serPort,90);
end
