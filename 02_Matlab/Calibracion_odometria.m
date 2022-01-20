%% Calibración de odometría - Regresión lineal (min^2)
% Calcula relación entre la desviación típica de Delta_x, Delta_y y 
% Delta_theta medidos por la odometría respecto a la veloxcidad
% Trabajo de Guiado y Navegación de Robots
% Pablo García Peris, Guillermo Illana Gisbert y Alberto López Rodríguez
clear
close all

global time_unit robot_name

variables_globales

%% Toma de datos
N_medidas = 100; % número de medidas para calcular la desviación típica

v = 0:0.1:1; % velocidad
w = 0:0.1:1; % vel. angular

N = length(v) * length(w);

var_odometria = zeros(N, 3);
variables = zeros(N, 3);

idx = 1;
for i = 1:length(v)
    for j = 1:length(w)

        apoloPlaceMRobot(robot_name,[5 2 0],0);

        apoloUpdate()
        
        odometria = zeros(N_medidas, 3);
        
        for k = 1:N_medidas
            % resetea odometría y mueve
            apoloResetOdometry(robot_name, [0 0 0],'World 1');
            apoloMoveMRobot(robot_name,[v(i) w(j)], time_unit, 'World 1');
            apoloUpdate();
            
            % obtiene los valores de la odometría
            odometria_next = apoloGetOdometry(robot_name,'World 1');
            odometria(k,:) = odometria_next;
        
        end
        
%         if v(i) == 1 && w(j) == 0
%             figure(112);
%             subplot(3, 1, 1)
%             hold on
%             plot(1:N_medidas, odometria(:, 1)/mean(odometria(:, 1)), 'LineWidth', 1.5);
%             plot(1:N_medidas, odometria(:, 2)/mean(odometria(:, 2)), 'LineWidth', 2.5);
%             plot(1:N_medidas, odometria(:, 3)/mean(odometria(:, 3)), 'LineWidth', 1.5);
%             grid on
%             xlabel('medida')
%             ylabel("$\Delta \bar{X}_o(k)$", 'Interpreter','latex');
%             legend('\Delta x_o(k) (m)', '\Delta y_o(k) (m)', '\Delta \theta_o(k) (rad)')
% 
%             text(-0.15, 0.85, 'a)', 'Units', 'normalized')
%         end
% 
%         if v(i) == 0 && w(j) == 1
%             figure(112);
%             subplot(3, 1, 2)
%             hold on
%             plot(1:N_medidas, odometria(:, 1)/mean(odometria(:, 1)), 'LineWidth', 2.5);
%             plot(1:N_medidas, odometria(:, 2)/mean(odometria(:, 2)), 'LineWidth', 1.5);
%             plot(1:N_medidas, odometria(:, 3)/mean(odometria(:, 3)), 'LineWidth', 1.5);
%             grid on
%             xlabel('medida')
%             ylabel("$\Delta \bar{X}_o(k)$", 'Interpreter','latex');
%             legend('\Delta x_o(k) (m)', '\Delta y_o(k) (m)', '\Delta \theta_o(k) (rad)')
% 
%             text(-0.15, 0.85, 'b)', 'Units', 'normalized')
%         end
% 
%         if v(i) == 1 && w(j) == 0.5
%             figure(112);
%             subplot(3, 1, 3)
%             hold on
%             plot(1:N_medidas, odometria(:, 1)/mean(odometria(:, 1)), 'LineWidth', 2.5);
%             plot(1:N_medidas, odometria(:, 2)/mean(odometria(:, 2)), 'LineWidth', 1.5);
%             plot(1:N_medidas, odometria(:, 3)/mean(odometria(:, 3)), 'LineWidth', 1.5);
%             grid on
%             xlabel('medida')
%             ylabel("$\Delta \bar{X}_o(k)$", 'Interpreter','latex');
%             legend('\Delta x_o(k) (m)', '\Delta y_o(k) (m)', '\Delta \theta_o(k) (rad)')
% 
%             text(-0.15, 0.85, 'c)', 'Units', 'normalized')
%         end
        corrcoef(odometria)
        
        var_odometria(idx, :) = var(odometria);
        variables(idx, :, :) = [v(i)      v(i)*sqrt(w(j)) w(j)].^2;
        idx = idx + 1;
    end
end


%% Regresión lineal 
% matriz que relaciona la dasviación típica de cada variable (incremental)
% de la odometría con la velocidad y la velocidad angular de consigna

Q_pu = zeros(3, 3); % matriz Q, pero por unidad de la velocidad correspondiente
for i = 1:3
    Q_pu(i, i) = variables(:, i)\var_odometria(:, i);
end


%% Dibujos
texto_variables = ["v^2 (m^2/s^2)", "v^2·\omega^2 (m^2·rad/s^4)", "\omega^2 (rad^2/s^2)"];
texto_odometria = ["Var(\Delta)x (m^2)", "Var(\Delta)y (m^2)", "Var(\Delta\theta^2) (rad^2)"];

for i = 1:3
    figure(i);
    hold on
    plot(variables(:, i), var_odometria(:, i), '.k', 'LineWidth', 1.5);
    plot([0 1], [0 Q_pu(i, i)], '-r', 'LineWidth', 1.5);
    xlabel(num2str(i) + " - " + num2str(j))
    xlabel(texto_variables(i));
    ylabel(texto_odometria(i));
end

save calibracion_odometria Q_pu







