%% Código Completo para Pruebas PID - Sistema Cuadricóptero
clc;
clear;
close all;

%% 1. Configuración Inicial
% Parámetros físicos del cuadricóptero (consistentes con PSO)
m = 1.0;          % Masa [kg]
g = 9.81;         % Gravedad [m/s²]
Ix = 0.1; Iy = 0.1; Iz = 0.2;  % Momentos de inercia [kg·m²]

% Condiciones iniciales
x0 = zeros(6,1);       % [x, y, z, ϕ, θ, ψ] - todas en cero
xdot0 = zeros(6,1);    % Velocidades iniciales - todas en cero
X0 = [x0; xdot0];      % Vector de estado inicial

% Tiempo de simulación
tspan = [0 10];        % 10 segundos (igual que en PSO)

% Valores deseados (consistentes con tu optimización PSO)
z_des = 1;        % Altitud deseada [m]
phi_des = 0;      % Ángulo de roll deseado [rad]
theta_des = 0;    % Ángulo de pitch deseado [rad]
psi_des = pi/4;   % Ángulo de yaw deseado [rad] (π/4 = 45°)

%% 2. Parámetros PID para las 30 pruebas
pid_params = [
    % Kp_z    Ki_z    Kd_z  Kp_phi Ki_phi Kd_phi Kp_theta Ki_theta Kd_theta Kp_psi Ki_psi Kd_psi
    12.4973  1.0059  5     10     0.1    2      10       0.001    2        10     0.001  1.2917;
    8.0734   1.006   5     9.9569 0.0462 0.9265 6.5143   0.025    1.5661   4.8837 0.0315 1.7334;
    8.3745   1.0061  5     7.3605 0.0449 1.5376 10       0.001    2        3.7552 0.0926 2;
    12.4973  1.0059  5     10     0.05   1.4673 10       0.1      2        10     0.001  1.2917;
    8.0734   1.006   5     9.9569 0.0462 0.9265 6.5143   0.025    1.5661   4.8837 0.0315 1.7334;
    8.3745   1.0061  5     7.3605 0.0449 1.5376 10       0.001    2        3.7552 0.0926 2;
    8.8109   1.006   5     9.8706 0.0947 2      3.685    0.001    0.3119   4.0386 0.0995 0.2407;
    8.29     1.0061  4.9965 9.9996 0.0648 1.7913 10      0.001    1.0489   2.0099 0.0307 1.2063;
    8.8044   1.006   5     9.9997 0.0615 0.5104 9.9358   0.0503   2        9.9984 0.001  0.7714;
    12.4973  1.0059  5     10     0.05   1.4673 10       0.1      2        10     0.001  1.2917;
    8.0734   1.006   5     9.9569 0.0462 0.9265 6.5143   0.025    1.5661   4.8837 0.0315 1.7334;
    8.3745   1.0061  5     7.3605 0.0449 1.5376 10       0.001    2        3.7552 0.0926 2;
    8.29     1.0061  4.9965 9.9996 0.0648 1.7913 10      0.001    1.0489   2.0099 0.0307 1.2063;
    8.8044   1.006   5     9.9997 0.0615 0.5104 9.9358   0.0503   2        9.9984 0.001  0.7714;
    8.081    1.0061  5     10     0.0451 1.7256 10       0.001    1.2168   5.6047 0.0985 0.5115;
    7.9205   1.006   5     7.4172 0.047  2      3.5663   0.0123   1.7703   9.9893 0.1    1.5466;
    8.6248   1.006   5     7.6123 0.0413 1.1642 3.9362   0.0279   0.7691   6.4244 0.001  0.8658;
    15       1.0059  5     4.9204 0.027  0.4779 9.9439   0.0348   0.9839   0.1204 0.003  1.6964;
    8.6607   1.0061  5     9.9987 0.0954 0.8808 9.9692   0.047    0.7851   5.6543 0.044  0.4283;
    9.1155   1.0062  5     10     0.001  1.8574 10       0.0868   0.8812   6.6009 0.001  1.6374;
    8.8887   1.006   5     6.1149 0.0345 0.8018 4.5611   0.0295   1.5766   7.9497 0.001  0.954;
    7.9404   1.006   5     9.9516 0.0676 2      9.9317   0.1      1.942    6.7623 0.0359 1.3565;
    7.4871   1.0061  5     9.9769 0.001  2      10       0.001    1.0405   6.9447 0.0694 1.805;
    15       1.0054  5     4.4642 0.0155 0.8282 4.7143   0.0333   1.5405   1.7491 0.061  0.4731;
    8.1195   1.006   5     7.5666 0.0325 1.8958 9.5006   0.1      1.9981   4.7582 0.0879 2;
    8.3682   1.006   5     6.707  0.0319 1.4412 3.0491   0.001    0.2327   7.0684 0.008  0.8427;
    8.1027   1.006   4.9853 9.288  0.0378 2      4.6254   0.0206   1.2529   10     0.001  0.1623;
    8.5171   1.0062  5     10     0.001  1.5205 7.3878   0.0312   0.7934   2.421  0.001  1.4681;
    8.008    1.0062  5     9.9993 0.001  1.9964 4.7726   0.0203   2        8.6213 0.0012 1.4899;
    8.5698   1.006   4.9995 5.6587 0.025  1.1393 10       0.1      1.016    3.4006 0.1    1.5727
];

%% 3. Configuración de Visualización
line_width = 1.5;
font_size = 12;

%% 4. Realización de las 30 Pruebas
num_pruebas = size(pid_params, 1);
results = struct('fitness', zeros(num_pruebas,1), ...
                 'settling_time', zeros(num_pruebas,1), ...
                 'overshoot', zeros(num_pruebas,1), ...
                 'steady_error', zeros(num_pruebas,1));

for i = 1:num_pruebas
    % Extraer parámetros para esta prueba
    Kp_z = pid_params(i,1); Ki_z = pid_params(i,2); Kd_z = pid_params(i,3);
    Kp_phi = pid_params(i,4); Ki_phi = pid_params(i,5); Kd_phi = pid_params(i,6);
    Kp_theta = pid_params(i,7); Ki_theta = pid_params(i,8); Kd_theta = pid_params(i,9);
    Kp_psi = pid_params(i,10); Ki_psi = pid_params(i,11); Kd_psi = pid_params(i,12);
    
    % Resetear variables persistentes para cada prueba
    clear quadrotor_dynamics;
    
    % Simulación del sistema
    [t, X] = ode45(@(t,X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
                      Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
                      Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
                      z_des, phi_des, theta_des, psi_des), tspan, X0);
    
    % Cálculo de métricas de desempeño
    [results.fitness(i), results.settling_time(i), ...
     results.overshoot(i), results.steady_error(i)] = ...
        calculate_performance(t, X, z_des, phi_des, theta_des, psi_des);
    
    % Mostrar progreso
    fprintf('Prueba %2d/30 - Fitness: %.4f - Tiempo Estabilización: %.2f s - Sobrepico: %.1f%%\n', ...
            i, results.fitness(i), results.settling_time(i), results.overshoot(i));
    
    % Graficar cada 5 pruebas
    if mod(i,5) == 0 || i == num_pruebas
        figure('Position', [100, 100, 1000, 800]);
        
        % Altitud
        subplot(2,2,1);
        plot(t, X(:,3), 'b', t, z_des*ones(size(t)), 'r--', 'LineWidth', line_width);
        title(['Prueba ', num2str(i), ' - Altitud (z)'], 'FontSize', font_size+2);
        xlabel('Tiempo [s]', 'FontSize', font_size);
        ylabel('Altitud [m]', 'FontSize', font_size);
        legend('Real', 'Deseado', 'Location', 'best');
        grid on;
        
        % Roll
        subplot(2,2,2);
        plot(t, X(:,4), 'b', t, phi_des*ones(size(t)), 'r--', 'LineWidth', line_width);
        title(['Prueba ', num2str(i), ' - Roll (\phi)'], 'FontSize', font_size+2);
        xlabel('Tiempo [s]', 'FontSize', font_size);
        ylabel('Ángulo [rad]', 'FontSize', font_size);
        grid on;
        
        % Pitch
        subplot(2,2,3);
        plot(t, X(:,5), 'b', t, theta_des*ones(size(t)), 'r--', 'LineWidth', line_width);
        title(['Prueba ', num2str(i), ' - Pitch (\theta)'], 'FontSize', font_size+2);
        xlabel('Tiempo [s]', 'FontSize', font_size);
        ylabel('Ángulo [rad]', 'FontSize', font_size);
        grid on;
        
        % Yaw
        subplot(2,2,4);
        plot(t, X(:,6), 'b', t, psi_des*ones(size(t)), 'r--', 'LineWidth', line_width);
        title(['Prueba ', num2str(i), ' - Yaw (\psi)'], 'FontSize', font_size+2);
        xlabel('Tiempo [s]', 'FontSize', font_size);
        ylabel('Ángulo [rad]', 'FontSize', font_size);
        grid on;
        
        sgtitle(['Resultados Prueba ', num2str(i)], 'FontSize', font_size+4);
    end
end

%% 5. Resultados Finales
% Tabla de resultados
fprintf('\n=== RESULTADOS FINALES ===\n');
fprintf('Prueba\tFitness\t\tTiempo Est.\tSobrepico\tError SSE\n');
fprintf('==========================================================\n');
for i = 1:num_pruebas
    fprintf('%d\t%.4f\t%.2f s\t\t%.1f%%\t\t%.4f\n', ...
            i, results.fitness(i), results.settling_time(i), ...
            results.overshoot(i), results.steady_error(i));
end

% Gráfico de evolución del fitness
figure('Position', [200, 200, 800, 400]);
plot(1:num_pruebas, results.fitness, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 8);
xlabel('Número de Prueba', 'FontSize', font_size);
ylabel('Fitness (ITAE)', 'FontSize', font_size);
title('Evolución del Fitness en las 30 Pruebas PID', 'FontSize', font_size+2);
grid on;

%% 6. Funciones Auxiliares
function dXdt = quadrotor_dynamics(~, X, m, g, Ix, Iy, Iz,...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
        z_des, phi_des, theta_des, psi_des)
    
    persistent integral_z integral_phi integral_theta integral_psi
    if isempty(integral_z)
        integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;
    end
    
    % Extraer estados
    pos = X(1:6);       % [x, y, z, ϕ, θ, ψ]
    vel = X(7:12);      % [dx, dy, dz, dϕ, dθ, dψ]
    
    % Cálculo de errores
    errores = [z_des - pos(3);    % Error altitud
              phi_des - pos(4);   % Error Roll
              theta_des - pos(5); % Error Pitch
              psi_des - pos(6)];  % Error Yaw
    
    % Anti-windup para las integrales
    max_int = 10;
    integral_z = max(min(integral_z + errores(1)), -max_int);
    integral_phi = max(min(integral_phi + errores(2)), -max_int);
    integral_theta = max(min(integral_theta + errores(3)), -max_int);
    integral_psi = max(min(integral_psi + errores(4)), -max_int);
    
    % Control PID con saturación (implementación corregida)
    max_U = 20;
    
    % Cálculo temporal de las acciones de control
    U1_unbounded = Kp_z*errores(1) + Ki_z*integral_z + Kd_z*(-vel(3));
    U2_unbounded = Kp_phi*errores(2) + Ki_phi*integral_phi + Kd_phi*(-vel(4));
    U3_unbounded = Kp_theta*errores(3) + Ki_theta*integral_theta + Kd_theta*(-vel(5));
    U4_unbounded = Kp_psi*errores(4) + Ki_psi*integral_psi + Kd_psi*(-vel(6));
    
    % Aplicar saturación
    U1 = max(min(U1_unbounded, max_U), -max_U);
    U2 = max(min(U2_unbounded, max_U), -max_U);
    U3 = max(min(U3_unbounded, max_U), -max_U);
    U4 = max(min(U4_unbounded, max_U), -max_U);
    
    % Dinámica traslacional
    acc_lin = [...
        (cos(pos(4))*sin(pos(5))*cos(pos(6)) + sin(pos(4))*sin(pos(6)))*U1/m;
        (cos(pos(4))*sin(pos(5))*sin(pos(6)) - sin(pos(4))*cos(pos(6)))*U1/m;
        (cos(pos(4))*cos(pos(5))*U1/m) - g];
    
    % Dinámica rotacional
    acc_ang = [...
        (U2 + (Iy - Iz)*vel(5)*vel(6))/Ix;
        (U3 + (Iz - Ix)*vel(4)*vel(6))/Iy;
        (U4 + (Ix - Iy)*vel(4)*vel(5))/Iz];
    
    % Vector de derivadas
    dXdt = [vel; acc_lin; acc_ang];
end

function [fitness, t_settling, overshoot, steady_error] = ...
         calculate_performance(t, X, z_des, phi_des, theta_des, psi_des)
    
    % Factores de ponderación (consistentes con PSO)
    wz = 1.0;    % Ponderación para altitud
    wang = 0.1;  % Ponderación para ángulos
    
    % Extraer estados
    z = X(:,3);
    phi = X(:,4);
    theta = X(:,5);
    psi = X(:,6);
    
    % Cálculo de errores
    error_z = abs(z_des - z);
    error_phi = abs(phi_des - phi);
    error_theta = abs(theta_des - theta);
    error_psi = abs(psi_des - psi);
    
    % Tiempo de establecimiento (2% de banda)
    idx_settled = find(error_z <= 0.02*z_des, 1);
    if isempty(idx_settled)
        t_settling = t(end);
    else
        t_settling = t(idx_settled);
    end
    
    % Sobrepico porcentual (solo para altitud)
    overshoot = max(0, (max(z) - z_des)/z_des * 100);
    
    % Error en estado estacionario (último 10% de la simulación)
    idx_steady = round(0.9*length(t)):length(t);
    steady_error = mean(error_z(idx_steady)) + mean(error_phi(idx_steady)) + ...
                   mean(error_theta(idx_steady)) + mean(error_psi(idx_steady));
    
    % Penalización por sobreimpulso (similar a PSO)
    overshoot_penalty = 0;
    if overshoot > 10  % Solo penaliza si es >10%
        overshoot_penalty = 50 * (overshoot/100);
    end
    
    % Cálculo de ITAE ponderado
    itae_z = wz * trapz(t, t.*error_z);
    itae_phi = wang * trapz(t, t.*error_phi);
    itae_theta = wang * trapz(t, t.*error_theta);
    itae_psi = wang * trapz(t, t.*error_psi);
    
    % Fitness total (misma fórmula que en PSO)
    fitness = itae_z + itae_phi + itae_theta + itae_psi + overshoot_penalty + 10*steady_error;
    
    % Limitar fitness máximo para evitar valores extremos
    max_fitness = 1e6;
    fitness = min(fitness, max_fitness);
end