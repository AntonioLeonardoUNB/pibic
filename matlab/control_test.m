load loadds.mat

% Create UR RTDE Client
robotIP = '127.0.0.1'; 

ur = urRTDEClient(robotIP, CobotName='universalUR3');
URDF_file = 'ur3.urdf';

% Controller Parameters
Ts = 0.01;              % Sampling time (s)
Kp = 1.0;               % Proportional gain for position error
duration = 2;           % Duration of control (seconds)
numSteps = duration / Ts;

% Parse URDF to enable forward kinematics and Jacobian generation 
robot = importrobot(URDF_file);
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81];
endEffector = 'tool0';  % Name of the end-effector in the URDF file

fileName = 'posicoes_robo(10).csv'; % Nome do arquivo
fileID = fopen(fileName, 'w');

try
    for k = 1:numSteps
        pose = readCartesianPose(ur);
        x = pose(4:6);
        x_T = x';

        q = readJointConfiguration(ur);

        fprintf(fileID, '%f,%f,%f\n', x(1), x(2), x(3));

        % Forward kinematics and Jacobian:
        J = geometricJacobian(robot, q', endEffector);

        % Kinematic control law
        jp = J(4:6, :)
        disp(cond(jp))
        jointVelocities = pinv(J(4:6, :)) * ds_lpv(x_T);
        jointVelocities_T = jointVelocities';

        sendSpeedJCommands(ur, jointVelocities_T, Acceleration=0.5);

        pause(Ts);
    end

    fclose(fileID); % Fechamento normal
catch ME
    fclose(fileID); % Garante fechamento do arquivo em caso de erro
    rethrow(ME);    % Opcional: relança o erro para diagnóstico
end