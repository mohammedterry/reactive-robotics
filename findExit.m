function findExit(serPort)
    initVariables();
    align2exit(serPort);
    go2exit(serPort);
end

function initVariables()
    global gap; gap = 1;
    global Front; Front = 100; 
    global F; F = 100;
    global R; R = 100;  
    global L; L = 100;
end

function align2exit(serPort)
    global F_Sonar; t = 1; threshold = 1;
    updateSensors(serPort);
    F_prev = F_Sonar; F_curr = F_Sonar; 
    
    while t < 360
        updateSensors(serPort);
        F_prev = F_curr; F_curr = F_Sonar; 
        change = abs(F_curr-F_prev)/t; 
        if change > threshold break; end        
        turnAngle(serPort, .5, 1);
        t = t + 1;
    end
    
    t = 0;
    while t < 360
        updateSensors(serPort);
        F_prev = F_curr; F_curr = F_Sonar; 
        change = abs(F_curr-F_prev)/t; 
        if change > threshold  break; end        
        turnAngle(serPort, .05, 1);
        t = t + 1;
    end
    disp(t);
    turnAngle(serPort, .1, -1*t/2);
end

function go2exit(serPort)
    global L_Sonar; global R_Sonar; global F_Sonar;
    updateSensors(serPort);
    while L_Sonar > .5 && R_Sonar > .5 
        if F_Sonar < .5 
            break 
        end
        updateSensors(serPort);
        travelDist(serPort,1,.1);
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