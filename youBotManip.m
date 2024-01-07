%% DESCRIPTION
%
% Bahcesehir University
% Department of Mechatronics Engineering
%
% MCH4001 Fundamentals of Robotics
% Fall 2023 Project
% MATLAB YouBot Manipulation
%
% Version 0.1
% Date: 10.11.2023
%
% Assoc. Prof. Dr. Berke Gur

%% START OF CODE

% Initialize
clc;clear all;close all;

fprintf('Program started \n');

client = RemoteAPIClient();
sim = client.require('sim');
fprintf('Connected to remote API server \n');

%% PREP FOR SIM

% Get object handles 
youbot = sim.getObject("/youBot");

nCube = 5;
cubeHand = zeros(nCube,1);

cubeHand(1) = sim.getObject("/Rectangle13[0]");
cubeHand(2) = sim.getObject("/Rectangle13[1]");
cubeHand(3) = sim.getObject("/Rectangle13[2]");
cubeHand(4) = sim.getObject("/Rectangle13[3]");
cubeHand(5) = sim.getObject("/Rectangle13[4]");

%Set youBot position
youbotInitPos = [0,0,+0.09575];
sim.setObjectPosition(youbot, youbotInitPos, sim.handle_world);

sim.setObjectColor(cubeHand(1), 0, sim.colorcomponent_ambient_diffuse, {0,0,0});
sim.setObjectColor(cubeHand(2), 0, sim.colorcomponent_ambient_diffuse, {1,0,0});
sim.setObjectColor(cubeHand(3), 0, sim.colorcomponent_ambient_diffuse, {0,1,0});
sim.setObjectColor(cubeHand(4), 0, sim.colorcomponent_ambient_diffuse, {0,0,1});
sim.setObjectColor(cubeHand(5), 0, sim.colorcomponent_ambient_diffuse, {1,1,1});


%Corner coordinates of platform
xMin = -0.1215; xMax = 0.1215;
yMin = -0.225; yMax = -0.1;
zMin = 0.1733; zMax = 0.1733;

for indCube = 1:nCube
    
    xPos = xMin + (xMax - xMin)*rand(1,1);
    yPos = yMin + (yMax - yMin)*rand(1,1);
    zPos = zMin;
    gamma = -pi + (pi-(-pi))*rand(1,1);
    sim.setObjectOrientation(cubeHand(indCube),[0, 0, gamma], sim.handle_world);
    sim.setObjectPosition(cubeHand(indCube), [xPos, yPos, zPos], sim.handle_world);

end
%% ACTUAL SIM LOOP

% Run a simulation in stepping mode:
sim.setStepping(true);
sim.startSimulation();

while 1
    t = sim.getSimulationTime();
    if t >= 300; break; end
    s = sprintf('Simulation time: %.2f [s] (simulation running synchronously to client, i.e. stepping)', t);
    fprintf('%s\n', s);
    sim.addLog(sim.verbosity_scriptinfos, s);
    sim.step();  % triggers next simulation step
end
sim.stopSimulation();

return
   
    
    % Enable the synchronous mode on the client
    %sim.simxSynchronous(clientID,true);
    
    % Start simulation
    %sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);
    %sim.startSimulation();

    %int objectHandle = sim.getObject(string objectPath, dict options = {})    
    %youbot = sim.getObject('/youBot');
    %[returnCode,youbot] = sim.simxGetObjectHandle(clientID, 'youBot', sim.simx_opmode_blocking);
    %[returnCode,cube1] = sim.simxGetObjectHandle(clientID, 'Rectangle1 3[0]', sim.simx_opmode_blocking);
    
%     [returnCode] = simSetObjectColor(int objectHandle, int index, int colorComponent, float* rgbData)
%     %% CODE FOR CALCUALTING TRAJECTORY
%     
%     %Define drone starting point
%     az0 = 2*pi*rand(1,1); %random starting azimuth angle in range [0, 360] degrees [rad]
%     el0 = pi/3 + pi/6*rand(1,1); %random starting elevation angle in range [60, 90] degrees [rad]
%     R0 = 5; %range to origin [m]
%     
%     dirVec0 = [cos(az0)*cos(el0)
%         sin(az0)*cos(el0)
%         sin(el0)];
%     
%     x0 = R0 * dirVec0;
%     
%     azf = 2*pi*rand(1,1); %random starting azimuth angle in range [0, 360] degrees [rad]
%     elf = 0; %random starting elevation angle in range [60, 90] degrees [rad]
%     Rf = 0.5*rand(1,1); %range to origin [m]
%     
%     dirVecf = [cos(azf)*cos(elf)
%         sin(azf)*cos(elf)
%         sin(elf)];
%     
%     xf = Rf * dirVecf;
%     
    dt = 0.05; %control loop period [s]
    tMax = 10; %duration of flight [s]
    tVec = [0:dt:tMax].'; %time vector [s]
    nT = length(tVec);
%     
%     deltaX = xf-x0;
%     xTraj_des = repmat(x0.',nT,1) + repmat(deltaX.',nT,1).*repmat(tVec,1,3);
%     
%     vel_des = deltaX/tMax;
%     
%     xTraj = zeros(nT,3);
%     xTraj(1,:) = x0.';
%     
%     vel = zeros(nT,3);
%     vel(1,:) = vel_des;
%     
%     vel_var = 1;
%     
%     % Off-line calculation of trajectory
%     for (indt = 2:nT),
%         vel_err = vel_var/2 - vel_var*rand(1,3);
%         vel(indt,:) = vel_des.' + vel_err;
%         xTraj(indt,:) = xTraj(indt-1,:) + vel(indt,:)*dt;
%     end
    
    %% ACTUAL SIM LOOP
    
    % Set initial iiwa position
    youbotInitPos = [0,0,+0.09575];
    %sim.simxSetObjectPosition(clientID, youbot, -1, youbotInitPos, sim.simx_opmode_blocking);
    sim.simxSetObjectPosition(clientID, youbot, -1, youbotInitPos, sim.simx_opmode_blocking);
    sim.setObjectPosition(youbot, youbotInitPos, sim.handle_world);
    %sim.simxSetObjectColor(cube1, nil, sim_colorcomponent_ambient_diffuse,{0,1,0});
    %Set initial drone position
    %sim.simxSetObjectPosition(clientID, target, -1, x0, sim.simx_opmode_blocking);
    %sim.simxSetObjectPosition(clientID, drone, -1, x0, sim.simx_opmode_blocking);
    
    % The actual sim loop
    for indt = 2:nT
        disp(['Current simulation time: ',num2str(tVec(indt)),' s.']);
        
        %sim.simxSynchronousTrigger(clientID);
    end
    
    % Stop the simulation
    sim.stopSimulation();
    
    % Now close the connection to CoppeliaSim
    %sim.simxFinish(clientID);
    
%else
%    disp('Failed connecting to remote API server');
%end

%sim.delete(); % call the destructor

disp('Program ended');
