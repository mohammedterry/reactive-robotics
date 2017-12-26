function reactiveExploration(serPort)
    avoidAttract(serPort);
    %kissFromAnyAngle(serPort);
    align2beacon(serPort);
end


function kissFromAnyAngle(serPort)
    global F; global L; global R; 
    R_wheel = 0.2; L_wheel = 0.2;
    e = 0.1;
    
    updateSensors(serPort);
    while abs(L-R) > e || abs(R-F) > e;
        SetDriveWheelsCreate(serPort, R_wheel, L_wheel)        
        updateSensors(serPort);
        R_wheel = R*e; L_wheel = L*e;
        angle = AngleSensorRoomba(serPort);
    end
	SetDriveWheelsCreate(serPort,0, 0);
    display('centralised');  disp(abs(L-R)); disp(abs(F-R));
    

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
        
%         if ~any(Camera) Camera = 0; end
%         L_wheel = .05*(R*R)+ 0.1*(Camera*Camera); 
%         R_wheel = .05*(L*L) - 0.1*(Camera*Camera);
%         disp(L_wheel); disp(R_wheel);
%         
        if F_Bump == true
            L_wheel = -0.4; R_wheel = -0.5;
        elseif L_Bump == true
            L_wheel = 0.5; R_wheel = -0.5;
        elseif R_Bump == true
            L_wheel = -0.5; R_wheel = 0.5;            
        end       
        SetDriveWheelsCreate(serPort, max(-.5,min(.5,R_wheel)), max(-.5,min(.5,L_wheel)))        
    end

    display('beacon found');  
end

function align2beacon(serPort)
    global Camera; global F; global L_Sonar;
    ready = false; t = 0; tolerance = 14; halfWay = 200;
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
                turnAngle(serPort, .1, 4);
            else
                if t > tolerance halfWay = t/2; end
                t = 0;
            end
            SetDriveWheelsCreate(serPort, .4,.28) %spin anticlockwise        
        end
    end
     turnAngle(serPort, .1, 90); 
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
