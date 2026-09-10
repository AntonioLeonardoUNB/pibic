% =========================================================================
% Script de Controle de Trajetória do UR3 utilizando Sistemas Dinâmicos (DS)
% e Cinemática Inversa Generalizada (GIK)
% =========================================================================

% Carrega a variável ds_lpv (modelo do sistema dinâmico) do workspace
% Esta parte depende do toolbox ds-opt devidamente configurado c:\Users\Quito\Downloads\Dados_organizados\Drawing2\Drawing2_ds_lpv.mat
Clear
load dslpv_exemplo.mat;

% -------------------------------------------------------------------------
% 1. Configuração da Conexão com o Robô 
% -------------------------------------------------------------------------
robotIP = '127.0.0.1';      % IP do robô (URSim localhost ou robô real)
ur = urRTDEClient(robotIP, CobotName='universalUR3');

% -------------------------------------------------------------------------
% 2. Configuração do Modelo Cinemático (RigidBodyTree)
% -------------------------------------------------------------------------
% Extrai o modelo de corpos rígidos da conexão para cálculo da cinemática
robot = ur.RigidBodyTree;
robot.DataFormat = 'row';   % Formato dos dados em linha para compatibilidade com o solver
robot.Gravity = [0 0 -9.81]; % Vetor gravidade apontando para Z negativo
endEffector = 'tool0';      % Define o efetuador final alvo da cinemática

% -------------------------------------------------------------------------
% 3. Preparação para Gravação de Dados (Log)
% -------------------------------------------------------------------------
baseName = 'posicoes_ds';
ext = '.csv';
i = 1;

% Loop para gerar um nome de arquivo único, evitando sobrescrever logs anteriores
while true
    fileName = sprintf('%s(%d)%s', baseName, i, ext);
    if ~isfile(fileName)
        break; 
    end
    i = i + 1;
end

% Abre o arquivo CSV em modo de escrita
fileID = fopen(fileName, 'w');

% -------------------------------------------------------------------------
% 4. Posicionamento Inicial do Robô
% -------------------------------------------------------------------------
% Configurações predefinidas de juntas articulares

initangles  = [-0.0979, -1.6671, -1.3811, -3.1863, -0.9459, 3.1051];

% Envia o robô para a posição inicial aguardando a conclusão (timeout de 7s)
[result, state] = sendJointConfigurationAndWait(ur, initangles, EndTime=7);

% -------------------------------------------------------------------------
% 5. Configuração do Solver de Cinemática Inversa Generalizada (GIK)
% -------------------------------------------------------------------------
gik = generalizedInverseKinematics( ...
    'RigidBodyTree', robot, ...
    'ConstraintInputs', {'position', 'joint'});

% Define a restrição de posição alvo para o efetuador final com tolerância de 1mm
positionConstraint = constraintPositionTarget(endEffector);
positionConstraint.PositionTolerance = 1e-3;

% Define as restrições baseadas nos limites físicos das juntas do robô
jointConstraint = constraintJointBounds(robot);
physicalJointBounds = jointConstraint.Bounds;

% Limiares de variação máxima permitida por junta (em radianos) entre os passos.
% O algoritmo tenta da menor variação até a maior para garantir movimentos suaves.
jointStepThresholds = deg2rad([2 5 10 15 20 30]);

% -------------------------------------------------------------------------
% 6. Loop Principal de Controle e Execução da Trajetória
% -------------------------------------------------------------------------
try
    % Loop de iterações para a evolução do sistema dinâmico
    for k = 1:numSteps
        
        % Lê a pose cartesiana atual do robô
        pose = readCartesianPose(ur);
        x = pose(4:6); % Extrai o vetor de translação [X, Y, Z]
        
        % Calcula o vetor de velocidade (v) avaliando o modelo LPV no ponto atual
        v = ds_lpv(x');
        h = 1e-2; % Passo de integração temporal
        
        % Integração numérica para prever o próximo ponto alvo (x_1)
        x_1 = x + h * v';
        for j = 1:50
            x_1 = x_1 + h * (ds_lpv(x_1'))';
        end
        
        % Grava a posição cartesiana atual no arquivo de log
        fprintf(fileID, '%f,%f,%f\n', x(1), x(2), x(3));
        
        % --- INÍCIO DA CINEMÁTICA INVERSA GENERALIZADA ---
        
        cur_jointangles = readJointConfiguration(ur); % Lê os ângulos atuais das juntas
        positionConstraint.TargetPosition = x_1;      % Atualiza o alvo espacial
        solutionFound = false;
        
        % Tenta encontrar uma solução de cinemática limitando o Delta máximo das juntas.
        % Isso evita que o robô faça rotações bruscas para alcançar pontos próximos.
        for thresholdIndex = 1:numel(jointStepThresholds)
            maxJointChange = jointStepThresholds(thresholdIndex);

            % Restringe a busca da GIK a um raio seguro ao redor da posição atual
            lowerBound = max(physicalJointBounds(:,1), cur_jointangles(:) - maxJointChange);
            upperBound = min(physicalJointBounds(:,2), cur_jointangles(:) + maxJointChange);
            jointConstraint.Bounds = [lowerBound, upperBound];

            % Executa o solver GIK com os limites dinâmicos
            [qCandidate, solInfo] = gik(cur_jointangles, positionConstraint, jointConstraint);

            % Valida a solução candidata verificando o erro real e limites estritos
            candidateTransform = getTransform(robot, qCandidate, endEffector);
            positionError = norm(candidateTransform(1:3,4) - x_1(:));
            respectsJointBounds = all(qCandidate(:) >= lowerBound - 1e-8) && ...
                                  all(qCandidate(:) <= upperBound + 1e-8);

            % Se a solução é válida e está dentro da tolerância, encerra a busca
            if strcmpi(solInfo.Status, 'success') && ...
               positionError <= positionConstraint.PositionTolerance && ...
               respectsJointBounds
                
                qSol = qCandidate;
                solutionFound = true;
                fprintf(['Passo %d: solucao encontrada com Delta q max = ', ...
                    '%.1f graus e erro de posicao = %.3f mm.\n'], ...
                    k, rad2deg(maxJointChange), 1000 * positionError);
                break;
            end
        end

        % Interrompe o programa caso o robô alcance uma singularidade ou alvo inalcançável
        if ~solutionFound
            error('control:NoContinuousIKSolution', ...
                ['Nenhuma solucao IK continua encontrada no passo %d ', ...
                'com Delta q max de ate %.1f graus.'], ...
                k, rad2deg(jointStepThresholds(end)));
        end

        % Exibe as informações do solver e comanda o robô para os novos ângulos
        disp(solInfo);
        sendJointConfigurationAndWait(ur, qSol);
        
        % --- FIM DA CINEMÁTICA INVERSA GENERALIZADA ---
        
    end
    
    % Fecha o arquivo de registro adequadamente ao fim da execução
    fclose(fileID);
    
catch ME
    % Garante que o arquivo CSV seja fechado evitando dados corrompidos em caso de erro
    fclose(fileID); 
    rethrow(ME);
end