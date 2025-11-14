real_data = readmatrix('posicoes_robo(14).csv');

plot3(real_data(:,1), real_data(:,2), real_data(:,3), 'b-', 'LineWidth', 2);
hold on
scatter3(real_data(1,1), real_data(1,2), real_data(1,3), 100, 'g', 'filled'); % ponto inicial
scatter3(real_data(end,1), real_data(end,2), real_data(end,3), 100, 'm', 'filled'); % ponto final
legend({'Trajetória Sintética','Attractor','Velocidades','Trajetória Real','Início Real','Fim Real'});
