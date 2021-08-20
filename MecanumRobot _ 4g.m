% Use this script as a template for your code
% You might have to make several copies of the script for the different subtasks
% Communication to v-rep is already incorporated
% To see the robot in action, please ensure that v-rep is running before you execute this script

% Handles:
%  - The handle called bodyFrame will return the robot's position in the inertial frame
%  - The motor handles give you access to the motor in v-rep
%  - No need for sensors at this point. Only robot position should serve as feedback

% For errors or questions send me an e-mail: nasser.gyagenda@uni-siegen.de

%% DO NOT CHANGE ANYTHING HERE - Setting up the remote api
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
connected = false;

if (clientID>-1)
    connected = true;
    disp('Connected to remote API server');
end

% Vehicle parameters
a = 0.158516;       % COM to any front/back wheels [m]
b = 0.228020;       % COM to any right/left wheels [m]
R = 0.050369;     % Wheel radius [m]

if(connected)
    % handles to wheel motors
    [returnCode,wheelFL]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_blocking);
    [returnCode,wheelFR]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_blocking);
    [returnCode,wheelRR]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_blocking);
    [returnCode,wheelRL]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_blocking);
    
    % handle to robot body reference frame
    [returnCode,bodyFrame]=vrep.simxGetObjectHandle(clientID,'body_frame',vrep.simx_opmode_blocking);
end

%% To Do
if(connected)
    velRef = [0; 0; 0]; % Body speeds [vx; vy; w]
  
    currentPosition=[2.1 0 0];
    
    [returnCode,currentPosition]=vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_streaming);
    [returnCode,currentOrientation]=vrep.simxGetObjectOrientation(clientID,bodyFrame,-1,vrep.simx_opmode_streaming)
    
    %PID
    KPx = 3;
    KIx = 2;
    KDx = 0.5;
  
    KPy = 3;
    KIy = 2;
    KDy = 0.5;
    
    KPh = 3;
    KIh = 2;
    KDh = 0.5;
    
   
    epsilon = 0.2;  %Tolerence 
    Ts= 0.5;        %Sample Interval 
    theta = 0;
    rotationAngle = theta * pi/180;  % Degree to radian
    motionRadius = 2.1;              %Radius of the circular path 
    n=5;

    %PID Transfer Function 
    ControllerX= pid(KPx, KIx, KDx,Ts);
    ControllerY= pid(KPy, KIy, KDy,Ts);
    ControllerH= pid(KPh, KIh, KDh,Ts);
        
    destination = [motionRadius*cos(rotationAngle) motionRadius*sin(rotationAngle) 0 ]; %Path 
   
    %Feedback     
    positionErrorx =  feedback (ControllerX, destination(1,1) - currentPosition(1,1));
    positionErrory =  feedback (ControllerY, destination(1,2) - currentPosition(1,2));
    positionErrorh =  feedback (ControllerH, theta - currentOrientation(1,3)* pi/180);

        
for theta = 0:10:180
    
    [returnCode,currentOrientation]=vrep.simxGetObjectOrientation(clientID,bodyFrame,-1,vrep.simx_opmode_buffer);
    [returnCode,currentPosition]=vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_buffer);
    
    rotationAngle = theta * pi/180;
  
    destination = [motionRadius*cos(rotationAngle) motionRadius*sin(rotationAngle) 0 ];
    
    %For Position x-coordinate and y-coordinate  
    while abs(destination(1,1)-currentPosition(1,1))>epsilon || abs(destination(1,2)-currentPosition(1,2))>epsilon
            %Response 
            Vx= (destination(1,1) - currentPosition(1,1))*step(positionErrorx);
            Vy= (destination(1,2) - currentPosition(1,2))*step(positionErrory);
            
            %Velocity reference 
            velRef = [Vx(1,1); Vy(1,1); 0];
            
            %Inverse Kinematic 
            Kinematic = [1 1 (a+b); -1 1 -(a+b); 1 1 -(a+b); -1 1 (a+b)] / R;
            
            %Velocity for wheel 
            w = Kinematic*velRef;

            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFL,w(1), vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFR,w(2), vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRR,w(3), vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRL,w(4), vrep.simx_opmode_blocking);

            [returnCode,currentPosition]=vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_buffer);            
            [returnCode,currentOrientation]=vrep.simxGetObjectOrientation(clientID,bodyFrame,-1,vrep.simx_opmode_buffer);
        
            if (currentPosition(1,1)) < epsilon && abs(destination(1,1))< epsilon
                break
            end
    end
    while abs(theta + 90 - abs(currentOrientation(1,3))* 180/pi )> 12
            %Response 
            Vtheta = (currentOrientation(1,3)* pi/180)*step(positionErrorh);
            %velocity refernce 
            velRef = [0; 0; Vtheta(1,1)];
            
            %Inverse Kinematic 
            Kinematic = [1 1 -(a+b); -1 1 (a+b); 1 1 (a+b); -1 1 -(a+b)] / R;
            
            w = Kinematic*velRef;    

            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFL,w(1), vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFR,w(2), vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRR,w(3), vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRL,w(4), vrep.simx_opmode_blocking);            
            
            [returnCode,currentOrientation]=vrep.simxGetObjectOrientation(clientID,bodyFrame,-1,vrep.simx_opmode_buffer);
            [returnCode,currentPosition]=vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_buffer);            
            
           
     end
     
end 
    %Robot Brake
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFL,0, vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFR,0, vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRR,0, vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRL,0, vrep.simx_opmode_blocking);
 
   
%% DO NOT CHANGE ANYTHING HERE 
% Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!