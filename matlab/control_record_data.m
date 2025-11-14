% Import the variable ds_lpv

load loadds.mat

% Create UR RTDE Client

robotIP = '127.0.0.1'; 
ur = urRTDEClient(robotIP, CobotName='universalUR3');
URDF_file = 'ur3.urdf';

% Controller Parameters
Ts = 0.01;              
Kp = 1.0;              
duration = 2;          
numSteps = duration / Ts;

% Parse URDF to enable forward kinematics and Jacobian generation 

robot = importrobot(URDF_file);
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81];
endEffector = 'tool0';  

% Preparing to log data

baseName = 'posicoes_robo';
ext = '.csv';
i = 1;

while true
    fileName = sprintf('%s(%d)%s', baseName, i, ext);
    if ~isfile(fileName)
        break; 
    end
    i = i + 1;
end

fileID = fopen(fileName, 'w');

% Setting the robot to initial position

initangles = [-1.6007,-1.7271,-2.2030,-0.8080,1.5951,-0.0310];
[result,state] = sendJointConfigurationAndWait(ur,initangles,EndTime=7);

% Control via numeric integration 

try
    for k = 1:6
        pose = readCartesianPose(ur);
        new_pose = pose + [0,0,0,0.006,0.005,0.01];
        [result,state] = sendCartesianPoseAndWait(ur, new_pose)
        fprintf(fileID, '%f,%f,%f\n', pose(4), pose(5), pose(6));
    end

    for k = 1:6
        pose = readCartesianPose(ur);
        new_pose = pose + [0,0,0,0.009,0.008,0.01];
        [result,state] = sendCartesianPoseAndWait(ur, new_pose)
        fprintf(fileID, '%f,%f,%f\n', pose(4), pose(5), pose(6));
    end
    
    for k = 1:6
        pose = readCartesianPose(ur);
        new_pose = pose + [0,0,0,0.012,0.011,0.01];
        [result,state] = sendCartesianPoseAndWait(ur, new_pose)
        fprintf(fileID, '%f,%f,%f\n', pose(4), pose(5), pose(6));
    end
    
    for k = 1:6
        pose = readCartesianPose(ur);
        new_pose = pose + [0,0,0,0.015,0.0014,0.01];
        [result,state] = sendCartesianPoseAndWait(ur, new_pose)
        fprintf(fileID, '%f,%f,%f\n', pose(4), pose(5), pose(6));
    end
    
    for k = 1:6
        pose = readCartesianPose(ur);
        new_pose = pose + [0,0,0,0.018,0.017,0.01];
        [result,state] = sendCartesianPoseAndWait(ur, new_pose)
        fprintf(fileID, '%f,%f,%f\n', pose(4), pose(5), pose(6));
    end

    for k = 1:6
        pose = readCartesianPose(ur);
        new_pose = pose + [0,0,0,0.021,0.020,0.01];
        [result,state] = sendCartesianPoseAndWait(ur, new_pose)
        fprintf(fileID, '%f,%f,%f\n', pose(4), pose(5), pose(6));
    end

    fclose(fileID); 

catch ME
    fclose(fileID); % Garante fechamento do arquivo em caso de erro
    rethrow(ME);    % Opcional: relança o erro para diagnóstico
end