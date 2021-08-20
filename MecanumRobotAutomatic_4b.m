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
    velRef = [0; 0; 0];   % Body speeds [vx; vy; w]
  
    currentPosition=[0 0 0];
    [returnCode,currentPosition]=vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_streaming);
  
    % PID Constants
    KPx = 4;
    KIx = 0.05;
    KDx = 0.09;
  
    KPy = KPx;
    KIy = KIx;
    KDy =  KDx;
    
    destination = [0 1 0];
    
    epsilon = 0.05;  %Tolerance 
    Ts= 0.8;         %Sample interval
    
   
    
   
    path =[0,1,0];
    index = 1;
    
    %PID Transfer Function 
    ControllerX= pid(KPx, KIx, KDx,Ts);
    ControllerY= pid(KPy, KIy, KDy,Ts);
        
    destination = [path(index,1), path(index,2), path(index,3)];
        
    positionErrorx =  feedback (ControllerX, destination(1,1) - currentPosition(1,1));
    positionErrory =  feedback (ControllerY, destination(1,2) - currentPosition(1,2));

       
  while abs(destination(1,1)-currentPosition(1,1))>epsilon || abs(destination(1,2)-currentPosition(1,2))>epsilon
        %Response 
        Vx= (destination(1,1) - currentPosition(1,1))*step(positionErrorx);
        Vy= (destination(1,2) - currentPosition(1,2))*step(positionErrory);
       
        velRef = [Vx(1,1); Vy(1,1); 0];   %Velocity reference 
       
       %Kinematic = [1 1 -(a+b); -1 1 (a+b); 1 1 (-a-b); -1 1 (a+b)] / R;  %Inverse Kinematic we depicted 
        Kinematic = [1 1 -(a+b); -1 1 (a+b); 1 1 (a+b); -1 1 -(a+b)] / R;  %Inverse Kinematic from circular we used 
       
        
        w = Kinematic*velRef;    %Angular Velocity 
        
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFL,w(1), vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFR,w(2), vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRR,w(3), vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRL,w(4), vrep.simx_opmode_blocking);
       
        [returnCode,currentPosition]=vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_buffer);
         currentPosition
         positionErrorx
         positionErrory
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