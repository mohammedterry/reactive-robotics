function perpendiculator(serPort)
    initVariables();
    perpendicularise(serPort, 1);
    kiss(serPort);
end

function initVariables()
    global gap; gap = 1;
    global Front; Front = 100; 
    global F; F = 100;
    global R; R = 100;  
    global L; L = 100;
end

function perpendicularise(serPort, turn)
    global F; F_curr = 100;
    
    while true
        turnAngle(serPort, .03, turn);
        updateSensors(serPort);
        F_prev = F_curr; F_curr = F; 
        if F_curr > F_prev break; end    
    end
    
    if turn > 0
        perpendicularise(serPort, -1);
        turnAngle(serPort, .01, 1);
    end
end

function kiss(serPort)
    global F;
    while F > .03
        updateSensors(serPort);
        travelDist(serPort,F,.01);       
    end
    travelDist(serPort,.2,-.5);       
end

function updateSensors(serPort)
    %INIT VARIABLES
    global distBeacon;  global L; global R; global F; global Camera; 
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
    disp('left...'); disp(L); disp('...front...'); disp(F); disp('...right'); disp(R); 
end

function turnAccurate(serPort, desiredAngle)
    turnAngle(serPort, .1, desiredAngle); 
    actualAngle = AngleSensorRoomba(serPort); actualAngle = ( actualAngle*180 ) /pi; %rad -> deg
    if (abs(actualAngle - desiredAngle) > .5) turnAccurate(serPort, desiredAngle - actualAngle); end
end