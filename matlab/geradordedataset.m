% Script para processar demonstração e salvar no formato esperado pelo ds-opt

% 1. Carregar dados de posição
data_positions = readmatrix('posicoes_robo(14).csv'); % Dados no formato [x, y, z]

% Pré-processamento
data_positions = unique(data_positions, 'rows', 'stable');

% 2. Configurações
dt = 0.050; % Intervalo de amostragem em segundos
[num_samples, num_dimensions] = size(data_positions);

% 3. Inicializar matriz de velocidades
data_velocities = zeros(num_samples, num_dimensions);

% 4. Calcular deslocamentos e velocidades
for i = 2:num_samples - 1 % Não processar o último ponto
    % Vetor deslocamento
    displacement = data_positions(i, :) - data_positions(i-1, :);
    
    % Velocidade no intervalo
    data_velocities(i, :) = displacement / dt;
end

% Para o primeiro ponto (onde não há anterior), replicar a velocidade do segundo
data_velocities(1, :) = data_velocities(2, :);

% Forçar as últimas velocidades a serem [0, 0, 0]
data_velocities(num_samples, :) = [0, 0, 0];

% 5. Empilhar posições e velocidades
% Combinar posições (primeiras linhas) e velocidades (últimas linhas)
data_combined = [data_positions'; data_velocities'];

% 6. Estruturar como cell array
data = {data_combined}; % Uma única demonstração

% 7. Salvar no formato .mat
save('demonstration6.mat', 'data');