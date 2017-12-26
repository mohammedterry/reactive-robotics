function main(serPort)
    initRobot();
    reactiveCentral(serPort);
    align2exit(serPort);
    go2exit(serPort);
    avoidAttract(serPort);
    align2beacon(serPort);
    perpendicularise(serPort, 1);
    kiss(serPort);
    attractiveWalls(serPort);
    mouse(serPort);
	reactiveCentral(serPort);
end

function initRobot()
    global Camera; Camera = 100; 
    global distBeacon; distBeacon = 100;
    global Front; Front = 100; 
    global F; F = 100;
    global R; R = 100;  
    global L; L = 100;
end

function reactiveCentral(serPort)
    global F; global L; global R; global F_Bump; global L_Bump; global R_Bump;
    e = 0.1; minimumDistance = 1.7;
    
    updateSensors(serPort);
    while ~(abs(L-R) < e && L > minimumDistance)
        updateSensors(serPort);
        R_wheel = L*e; L_wheel = R*e;
        
        if F_Bump == true
            L_wheel = -0.4; R_wheel = -0.5;
        elseif L_Bump == true
            L_wheel = 0.5; R_wheel = -0.5;
        elseif R_Bump == true
            L_wheel = -0.5; R_wheel = 0.5;            
        end           
        SetDriveWheelsCreate(serPort, R_wheel, L_wheel)        
    end
    SetDriveWheelsCreate(serPort,0, 0);      
    display('centralised');  disp(abs(L-R)); disp(abs(F-R));
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
    global F_Sonar;
    updateSensors(serPort);
    while F_Sonar > 1 
        updateSensors(serPort);
        travelDist(serPort,1,.1);
    end
end


function avoidAttract(serPort)
   global L; global R; global Camera; global F_Bump; global L_Bump; global R_Bump; global distBeacon;
    A = .5; radius = 2;
    while true  
        updateSensors(serPort);
        if distBeacon < radius break; end
        
        if  Camera >= 0 % + means turn L
            disp(Camera);
            L_wheel = A*(R*R)- A*Camera; 
            R_wheel = A*(L*L);
        elseif Camera < 0 %- means turn R
             disp(Camera);
             L_wheel = A*(R*R); 
             R_wheel = A*(L*L)+ A*Camera;           
        else
            L_wheel = A*(R*R); 
            R_wheel = A*(L*L);
        end
        disp(Camera);
         
        if F_Bump == true
            L_wheel = -0.4; R_wheel = -0.5;
        elseif L_Bump == true
            L_wheel = 0.5; R_wheel = -0.5;
        elseif R_Bump == true
            L_wheel = -0.5; R_wheel = 0.5;            
        end       
        SetDriveWheelsCreate(serPort, max(-.5,min(.5,R_wheel)), max(-.5,min(.5,L_wheel)))        
    end
end

function align2beacon(serPort)
    global Camera; global F; global L_Sonar;
    ready = false; t = 0; halfWay = 200; tolerance = 30;
    while ready == false;
        while any(Camera)
            t = 0; 
            updateSensors(serPort);
            SetDriveWheelsCreate(serPort, .28,.4) %spin clockwise        
        end
        while ~any(Camera)
            updateSensors(serPort);
            if L_Sonar > .4 && L_Sonar < 1.6
                t = t+1; disp(t); disp(halfWay);
                if t >= halfWay-1
                    ready = true; break;
                end
            elseif F < .5  
                disp('wall blocking way!'); 
                t = 0; 
                turnAngle(serPort, .1, 2);
            else
                if t > tolerance halfWay = t/2; end
                t = 0;
            end
            SetDriveWheelsCreate(serPort, .4,.28) %spin anticlockwise        
        end
    end
     turnAngle(serPort, .1, 90); 
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
    turnAngle(serPort, .1, 180);
end

function attractiveWalls(serPort)
    global F; global L; global R; 
    R_wheel = 0.2; L_wheel = 0.2;
    e = 0.1;
    
    updateSensors(serPort);
    while F > .1;
        SetDriveWheelsCreate(serPort, R_wheel, L_wheel)        
        updateSensors(serPort);
        R_wheel = R*e; L_wheel = L*e;
    end
    turnAngle(serPort, .1, 90);
end

function mouse(serPort)
    global F_Sonar; global L; global R; 
    wrongWall = true; threshold = 3; A = .3; 
    %to reset odometer
    turnAngle(serPort,.1,-.5); direction = AngleSensorRoomba(serPort); direction = 0;
    
    while true
        updateSensors(serPort);  
        
        if L < A || R > A
            SetDriveWheelsCreate(serPort, .1,.4) %spin clockwise  .1.4   
        elseif L > A || R < A
            SetDriveWheelsCreate(serPort, .4,.1) %spin anticlockwise .4.1    
        else 
            SetDriveWheelsCreate(serPort, .3,.3) %spin anticlockwise     
        end
        
        direction = direction + AngleSensorRoomba(serPort);
        disp(direction);
        if direction > threshold  %+ means going clockwise - ergo...around outter wall!!!
            disp('going clockwise - must be outer wall');
            wrongWall = true;
            break;
        elseif direction < -1*threshold  %- means going anticlockwise - ergo..around home!
            disp('going anticlockwise - must be house');
            wrongWall = false;
            break;
        end
    end
    disp('test complete'); 
    
    %get away from this wall and onto the other one!
    if wrongWall == true
        turnAngle(serPort,.1,40);
        travelDist(serPort,1,2);
        while F_Sonar > 10 
             updateSensors(serPort);
             travelDist(serPort,1,.1);
        end
        attractiveWalls(serPort);
        mouse(serPort);
    else
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
end


function updateSensors(serPort)
    %INIT VARIABLES
    global distBeacon;  global L; global R; global F; global Camera; global F_Bump; global L_Bump; global R_Bump; global F_Sonar; global R_Sonar; global L_Sonar;
    R_Bump = false; L_Bump = false; F_Bump = false; L_Sonar = 100; R_Sonar = 100; F_Sonar = 100; L_Lidar = 100; R_Lidar = 100; F_Lidar = 100;    
    %INIT BUMP SENSORS
    [R_Bump, L_Bump, ~, ~, ~, F_Bump] = BumpsWheelDropsSensorsRoomba(serPort);   
    %INIT CAMERA SENSORS    
    Camera = CameraSensorCreate(serPort); [~, distBeacon, colorBeacon] = CameraSensorCreate(serPort);
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