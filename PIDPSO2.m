%% 1. Configuración inicial
clear all; close all; clc;

%% 2. Ejecutar optimización PSO
[global_best, convergence] = optimize_pid_with_pso();

%% 3. Mostrar resultados y simular con ganancias óptimas
disp('=== Mejores ganancias encontradas ===');
fprintf('Altura (z): Kp=%.4f, Ki=%.4f, Kd=%.4f\n', global_best.position(1:3));
fprintf('Roll (φ):   Kp=%.4f, Ki=%.4f, Kd=%.4f\n', global_best.position(4:6));
fprintf('Pitch (θ):  Kp=%.4f, Ki=%.4f, Kd=%.4f\n', global_best.position(7:9));
fprintf('Yaw (ψ):    Kp=%.4f, Ki=%.4f, Kd=%.4f\n', global_best.position(10:12));

% Graficar convergencia
figure;
plot(convergence, 'LineWidth', 2);
xlabel('Iteración'); ylabel('Fitness (ITAE ponderado)');
title('Convergencia del PSO'); grid on;

%% 4. Simulación con ganancias óptimas
simulate_with_optimal_gains(global_best.position);

%% ========== FUNCIONES ========== 

%% Función principal de optimización PSO
function [global_best, B] = optimize_pid_with_pso()
    % Parámetros del PSO ajustados
    nVar = 12; % 4 controladores × 3 parámetros
    VarMin = [2.0  0.01  0.5 ...   % Límites para z (Kp, Ki, Kd)
              0.5  0.001 0.5 ...   % para phi
              0.5  0.001 0.5 ...   % para theta
              0.1  0.001 0.1];     % para psi
          
    VarMax = [8.0  0.2   3.0 ...   % Límites para z
              5.0  0.05  2.0 ...   % para phi
              5.0  0.05  2.0 ...   % para theta
              2.0  0.01  1.0];     % para psi

    MaxIter = 30;
    nPop = 15;
    w = 0.9;       % Inercia inicial reducida
    w_damp = 0.99; % Factor de decremento
    c1 = 1.8;      % Cognitivo
    c2 = 2.0;      % Social

    % Inicialización
    empty_particle.position = [];
    empty_particle.velocity = [];
    empty_particle.fitness = [];
    empty_particle.best.position = [];
    empty_particle.best.fitness = [];
    
    pop = repmat(empty_particle, nPop, 1);
    global_best.fitness = inf;
    B = zeros(MaxIter, 1);

    % Población inicial
    for i = 1:nPop
        pop(i).position = unifrnd(VarMin, VarMax);
        pop(i).velocity = zeros(1, nVar);
        pop(i).fitness = pid_objective_function(pop(i).position);
        pop(i).best = pop(i);
        
        if pop(i).fitness < global_best.fitness
            global_best = pop(i).best;
        end
    end

    % Bucle principal
    for iter = 1:MaxIter
        for i = 1:nPop
            % Actualizar velocidad
            r1 = rand(1, nVar);
            r2 = rand(1, nVar);
            
            pop(i).velocity = w * pop(i).velocity + ...
                             c1 * r1 .* (pop(i).best.position - pop(i).position) + ...
                             c2 * r2 .* (global_best.position - pop(i).position);
            
            % Actualizar posición
            pop(i).position = pop(i).position + pop(i).velocity;
            
            % Aplicar límites
            pop(i).position = max(pop(i).position, VarMin);
            pop(i).position = min(pop(i).position, VarMax);
            
            % Evaluar
            pop(i).fitness = pid_objective_function(pop(i).position);
            
            % Actualizar mejores
            if pop(i).fitness < pop(i).best.fitness
                pop(i).best.position = pop(i).position;
                pop(i).best.fitness = pop(i).fitness;
                
                if pop(i).best.fitness < global_best.fitness
                    global_best = pop(i).best;
                end
            end
        end
        
        % Actualizar inercia
        w = w * w_damp;
        
        % Guardar histórico
        B(iter) = global_best.fitness;
        fprintf('Iter %d: Mejor Fitness = %.4f\n', iter, B(iter));
    end
end

%% Función objetivo mejorada
function fitness = pid_objective_function(ganancias)
    % Parámetros físicos
    m = 1.0; g = 9.81;
    Ix = 0.1; Iy = 0.1; Iz = 0.2;
    
    % Condiciones iniciales
    x0 = zeros(6,1); xdot0 = zeros(6,1);
    X0 = [x0; xdot0];
    tspan = [0 15];
    
    % Referencias
    z_des = 1; 
    phi_des = pi/6;
    theta_des = pi/6;
    psi_des = 0;
    
    % Desempaquetar ganancias
    Kp_z = ganancias(1); Ki_z = ganancias(2); Kd_z = ganancias(3);
    Kp_phi = ganancias(4); Ki_phi = ganancias(5); Kd_phi = ganancias(6);
    Kp_theta = ganancias(7); Ki_theta = ganancias(8); Kd_theta = ganancias(9);
    Kp_psi = ganancias(10); Ki_psi = ganancias(11); Kd_psi = ganancias(12);
    
    % Simulación
    try
        [t, X] = ode45(@(t,X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
                          Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
                          Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
                          z_des, phi_des, theta_des, psi_des), tspan, X0);
        
        % Cálculo de errores (ITAE ponderado)
        error_z = abs(z_des - X(:,3));
        error_phi = abs(phi_des - X(:,4));
        error_theta = abs(theta_des - X(:,5));
        error_psi = abs(psi_des - X(:,6));
        
        % Pesos para balancear el control
        weight_z = 1.0;
        weight_angle = 1.5;
        
        % Fitness final
        fitness = weight_z * trapz(t, t.*error_z) + ...
                 weight_angle * (trapz(t, t.*error_phi) + ...
                               trapz(t, t.*error_theta) + ...
                               trapz(t, t.*error_psi));
             
    catch
        fitness = 1e6; % Penalización alta si falla la simulación
    end
end

%% Dinámica del quadrotor (versión corregida)
function dXdt = quadrotor_dynamics(~, X, m, g, Ix, Iy, Iz,...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
        z_des, phi_des, theta_des, psi_des)
    
    persistent integral_z integral_phi integral_theta integral_psi
    if isempty(integral_z)
        integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;
    end
    
    % Estados actuales
    pos = X(1:6);
    vel = X(7:12);
    
    % Errores
    e_z = z_des - pos(3);
    e_phi = phi_des - pos(4);
    e_theta = theta_des - pos(5);
    e_psi = psi_des - pos(6);
    
    % Anti-windup con clamping
    max_int = 5;
    integral_z = max(min(integral_z + e_z, max_int), -max_int);
    integral_phi = max(min(integral_phi + e_phi, max_int), -max_int);
    integral_theta = max(min(integral_theta + e_theta, max_int), -max_int);
    integral_psi = max(min(integral_psi + e_psi, max_int), -max_int);
    
    % Control PID con saturación
    U1 = min(max(Kp_z*e_z + Ki_z*integral_z + Kd_z*(-vel(3)), 0), 25);
    U2 = Kp_phi*e_phi + Ki_phi*integral_phi + Kd_phi*(-vel(4));
    U3 = Kp_theta*e_theta + Ki_theta*integral_theta + Kd_theta*(-vel(5));
    U4 = Kp_psi*e_psi + Ki_psi*integral_psi + Kd_psi*(-vel(6));
    
    % Dinámica traslacional
    acc_lin = [
        (cos(pos(4))*sin(pos(5))*cos(pos(6)) + sin(pos(4))*sin(pos(6)))*U1/m;
        (cos(pos(4))*sin(pos(5))*sin(pos(6)) - sin(pos(4))*cos(pos(6)))*U1/m;
        (cos(pos(4))*cos(pos(5))*U1/m) - g
    ];
    
    % Dinámica rotacional
    acc_ang = [
        (U2 + (Iy - Iz)*vel(5)*vel(6))/Ix;
        (U3 + (Iz - Ix)*vel(4)*vel(6))/Iy;
        (U4 + (Ix - Iy)*vel(4)*vel(5))/Iz
    ];
    
    dXdt = [vel; acc_lin; acc_ang];
end

%% Función para calcular señales de control
function U = calculate_control(X, t, m, g, Ix, Iy, Iz, Kp_z, Ki_z, Kd_z, ...
                             Kp_phi, Ki_phi, Kd_phi, Kp_theta, Ki_theta, Kd_theta, ...
                             Kp_psi, Ki_psi, Kd_psi, z_des, phi_des, theta_des, psi_des)
    
    persistent integral_z integral_phi integral_theta integral_psi
    if isempty(integral_z)
        integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;
    end
    
    % Estados actuales
    pos = X(1:6);
    vel = X(7:12);
    
    % Errores
    e_z = z_des - pos(3);
    e_phi = phi_des - pos(4);
    e_theta = theta_des - pos(5);
    e_psi = psi_des - pos(6);
    
    % Anti-windup con clamping
    max_int = 5;
    integral_z = max(min(integral_z + e_z, max_int), -max_int);
    integral_phi = max(min(integral_phi + e_phi, max_int), -max_int);
    integral_theta = max(min(integral_theta + e_theta, max_int), -max_int);
    integral_psi = max(min(integral_psi + e_psi, max_int), -max_int);
    
    % Control PID con saturación
    U1 = min(max(Kp_z*e_z + Ki_z*integral_z + Kd_z*(-vel(3)), 0), 25);
    U2 = Kp_phi*e_phi + Ki_phi*integral_phi + Kd_phi*(-vel(4));
    U3 = Kp_theta*e_theta + Ki_theta*integral_theta + Kd_theta*(-vel(5));
    U4 = Kp_psi*e_psi + Ki_psi*integral_psi + Kd_psi*(-vel(6));
    
    U = [U1, U2, U3, U4];
end

%% Función de simulación con ganancias óptimas
function simulate_with_optimal_gains(optimal_gains)
    % Asignar ganancias
    Kp_z = optimal_gains(1); Ki_z = optimal_gains(2); Kd_z = optimal_gains(3);
    Kp_phi = optimal_gains(4); Ki_phi = optimal_gains(5); Kd_phi = optimal_gains(6);
    Kp_theta = optimal_gains(7); Ki_theta = optimal_gains(8); Kd_theta = optimal_gains(9);
    Kp_psi = optimal_gains(10); Ki_psi = optimal_gains(11); Kd_psi = optimal_gains(12);
    
    % Parámetros del sistema
    m = 1.0; g = 9.81; Ix = 0.1; Iy = 0.1; Iz = 0.2;
    x0 = zeros(6,1); xdot0 = zeros(6,1); X0 = [x0; xdot0];
    tspan = [0 15];
    
    % Referencias
    z_des = 1; 
    phi_des = pi/6; 
    theta_des = pi/6;
    psi_des = 0;
    
    % Simular
    [t, X] = ode45(@(t,X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
                      Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
                      Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
                      z_des, phi_des, theta_des, psi_des), tspan, X0);
    
    % Calcular señales de control a posteriori
    U = zeros(length(t), 4);
    for i = 1:length(t)
        U(i,:) = calculate_control(X(i,:)', t(i), m, g, Ix, Iy, Iz,...
                    Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
                    Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
                    z_des, phi_des, theta_des, psi_des);
    end
    
    % Graficar resultados
    figure('Position', [100 100 1200 800]);
    
    % Altura
    subplot(3,2,1);
    plot(t, X(:,3), 'b', t, z_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
    title('Altitud (z)'); xlabel('Tiempo (s)'); ylabel('m');
    legend('Real', 'Deseado'); grid on;
    
    % Ángulos
    subplot(3,2,2);
    plot(t, X(:,4), 'b', t, phi_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
    title('Roll (φ)'); xlabel('Tiempo (s)'); ylabel('rad'); grid on;
    
    subplot(3,2,3);
    plot(t, X(:,5), 'b', t, theta_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
    title('Pitch (θ)'); xlabel('Tiempo (s)'); ylabel('rad'); grid on;
    
    subplot(3,2,4);
    plot(t, X(:,6), 'b', t, psi_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
    title('Yaw (ψ)'); xlabel('Tiempo (s)'); ylabel('rad'); grid on;
    
    % Señales de control
    subplot(3,2,5);
    plot(t, U(:,1), 'LineWidth', 1.5);
    title('Fuerza Total (U1)'); xlabel('Tiempo (s)'); ylabel('N'); grid on;
    ylim([0 30]);
    
    subplot(3,2,6);
    plot(t, U(:,2:4), 'LineWidth', 1.5);
    title('Momentos de Control'); xlabel('Tiempo (s)'); ylabel('N·m');
    legend('U2 (Roll)', 'U3 (Pitch)', 'U4 (Yaw)'); grid on;
end