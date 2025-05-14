%% Configuración inicial del sistema
clear all; close all; clc;

% Parámetros físicos del cuadrotor
m = 1.0;          % Masa (kg)
g = 9.81;         % Gravedad (m/s²)
Ix = 0.1; Iy = 0.1; Iz = 0.2;  % Momentos de inercia (kg·m²)

% Condiciones iniciales
x0 = [0; 0; 0; 0; 0; 0];       % Posición [x, y, z, ϕ, θ, ψ]
xdot0 = [0; 0; 0; 0; 0; 0];    % Velocidades [dx, dy, dz, dϕ, dθ, dψ]

% Configuración de pruebas
num_pruebas = 30;  % Número de pruebas aleatorias a realizar
z_des = 1;         % Altitud deseada
phi_des = 0.2;       % Roll deseado
theta_des = 0.1;     % Pitch deseado
psi_des = 0;       % Yaw deseado
tspan = [0 10];    % Tiempo de simulación

% Rangos para parámetros PID según Ziegler-Nichols (modificados para tu caso)
rangos_pid = {
    % Eje       Kp_range        Ki_range        Kd_range
    'Altitud',  [8.0, 12.0],   [0.5, 1.0],     [4.0, 6.0];
    'Roll',     [1.5, 3.0],     [0.01, 0.05],   [0.1, 0.3];
    'Pitch',    [1.5, 3.0],     [0.01, 0.05],   [0.1, 0.3];
    'Yaw',      [0.5, 1.5],     [0.001, 0.01],  [0.05, 0.2];
};

% Prepara tabla para almacenar resultados
resultados = table();

%% Bucle de pruebas aleatorias
for prueba_actual = 1:num_pruebas
    fprintf('Ejecutando prueba %d/%d...\n', prueba_actual, num_pruebas);
    
    % Generar parámetros PID aleatorios dentro de los rangos especificados
    Kp_z = rangos_pid{1,2}(1) + (rangos_pid{1,2}(2)-rangos_pid{1,2}(1))*rand();
    Ki_z = rangos_pid{1,3}(1) + (rangos_pid{1,3}(2)-rangos_pid{1,3}(1))*rand();
    Kd_z = rangos_pid{1,4}(1) + (rangos_pid{1,4}(2)-rangos_pid{1,4}(1))*rand();
    
    Kp_phi = rangos_pid{2,2}(1) + (rangos_pid{2,2}(2)-rangos_pid{2,2}(1))*rand();
    Ki_phi = rangos_pid{2,3}(1) + (rangos_pid{2,3}(2)-rangos_pid{2,3}(1))*rand();
    Kd_phi = rangos_pid{2,4}(1) + (rangos_pid{2,4}(2)-rangos_pid{2,4}(1))*rand();
    
    Kp_theta = rangos_pid{3,2}(1) + (rangos_pid{3,2}(2)-rangos_pid{3,2}(1))*rand();
    Ki_theta = rangos_pid{3,3}(1) + (rangos_pid{3,3}(2)-rangos_pid{3,3}(1))*rand();
    Kd_theta = rangos_pid{3,4}(1) + (rangos_pid{3,4}(2)-rangos_pid{3,4}(1))*rand();
    
    Kp_psi = rangos_pid{4,2}(1) + (rangos_pid{4,2}(2)-rangos_pid{4,2}(1))*rand();
    Ki_psi = rangos_pid{4,3}(1) + (rangos_pid{4,3}(2)-rangos_pid{4,3}(1))*rand();
    Kd_psi = rangos_pid{4,4}(1) + (rangos_pid{4,4}(2)-rangos_pid{4,4}(1))*rand();
    
    % Estado inicial
    X0 = [x0; xdot0];
    
    % Variables globales para integrales PID
    global integral_z integral_phi integral_theta integral_psi;
    integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;
    
    % Simulación
    [t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
        z_des, phi_des, theta_des, psi_des), tspan, X0);
    
    % Vectores de referencia
    z_ref = z_des * ones(size(t));
    phi_ref = phi_des * ones(size(t));
    theta_ref = theta_des * ones(size(t));
    psi_ref = psi_des * ones(size(t));
    
    % Cálculo de métricas
    [IAE_z, ITSE_z, overshoot_z, settling_time_z] = calculate_metrics(t, X(:,3), z_ref);
    [IAE_phi, ~, overshoot_phi, settling_time_phi] = calculate_metrics(t, X(:,4), phi_ref);
    [IAE_theta, ~, overshoot_theta, settling_time_theta] = calculate_metrics(t, X(:,5), theta_ref);
    [IAE_psi, ~, overshoot_psi, settling_time_psi] = calculate_metrics(t, X(:,6), psi_ref);
    
    % Fitness combinado (promedio normalizado de métricas)
    fitness = mean([IAE_z, IAE_phi, IAE_theta, IAE_psi]);
    
    % Almacenar resultados
    new_row = table(...
        Kp_z, Ki_z, Kd_z, ...
        Kp_phi, Ki_phi, Kd_phi, ...
        Kp_theta, Ki_theta, Kd_theta, ...
        Kp_psi, Ki_psi, Kd_psi, ...
        fitness, IAE_z, ITSE_z, overshoot_z, settling_time_z, ...
        IAE_phi, overshoot_phi, settling_time_phi, ...
        IAE_theta, overshoot_theta, settling_time_theta, ...
        IAE_psi, overshoot_psi, settling_time_psi, ...
        'VariableNames', {...
        'Altitud_Kp', 'Altitud_Ki', 'Altitud_Kd', ...
        'Roll_Kp', 'Roll_Ki', 'Roll_Kd', ...
        'Pitch_Kp', 'Pitch_Ki', 'Pitch_Kd', ...
        'Yaw_Kp', 'Yaw_Ki', 'Yaw_Kd', ...
        'Fitness', 'IAE_z', 'ITSE_z', 'Overshoot_z', 'Settling_Time_z', ...
        'IAE_phi', 'Overshoot_phi', 'Settling_Time_phi', ...
        'IAE_theta', 'Overshoot_theta', 'Settling_Time_theta', ...
        'IAE_psi', 'Overshoot_psi', 'Settling_Time_psi'});
    
    resultados = [resultados; new_row];
    
    % Opcional: Graficar cada X pruebas (para no saturar)
    if mod(prueba_actual, 5) == 0
        figure('Name', sprintf('Prueba %d (Fitness=%.2f)', prueba_actual, fitness));
        subplot(2,2,1);
        plot(t, X(:,3), 'b', t, z_ref, 'r--', 'LineWidth', 1.5);
        title('Altitud');
        legend('Obtenido', 'Deseado');
        
        subplot(2,2,2);
        plot(t, X(:,4), 'b', t, phi_ref, 'r--', 'LineWidth', 1.5);
        title('Roll');
        
        subplot(2,2,3);
        plot(t, X(:,5), 'b', t, theta_ref, 'r--', 'LineWidth', 1.5);
        title('Pitch');
        
        subplot(2,2,4);
        plot(t, X(:,6), 'b', t, psi_ref, 'r--', 'LineWidth', 1.5);
        title('Yaw');
    end
end

%% Exportar resultados a Excel
writetable(resultados, 'Resultados_PID_Aleatorios.xlsx');
fprintf('Resultados guardados en "Resultados_PID_Aleatorios.xlsx"\n');

%% Funciones auxiliares
function [IAE, ITSE, overshoot, settling_time] = calculate_metrics(t, y, y_ref)
    error = y_ref - y;
    IAE = trapz(t, abs(error));
    ITSE = trapz(t, t.*error.^2);
    overshoot = max(0, (max(y) - y_ref(1))) / y_ref(1) * 100;
    settling_time = find_settling_time(t, y, y_ref(1), 0.02); % 2% de tolerancia
end

function st = find_settling_time(t, y, y_final, tolerance)
    idx = find(abs(y - y_final) > tolerance * abs(y_final), 1, 'last');
    if isempty(idx)
        st = t(end);
    else
        st = t(min(idx+1, length(t)));
    end
end

function dXdt = quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
        z_des, phi_des, theta_des, psi_des)
    
    global integral_z integral_phi integral_theta integral_psi;
    
    % Estados actuales
    pos = X(1:6);       % [x, y, z, ϕ, θ, ψ]
    vel = X(7:12);      % Velocidades
    
    % Errores
    errores = [
        z_des - pos(3);
        phi_des - pos(4);
        theta_des - pos(5);
        psi_des - pos(6)
    ];
    
    % Actualizar integrales (anti-windup opcional)
    integral_z = integral_z + errores(1);
    integral_phi = integral_phi + errores(2);
    integral_theta = integral_theta + errores(3);
    integral_psi = integral_psi + errores(4);
    
    % Ley de control PID
    U1 = Kp_z * errores(1) + Ki_z * integral_z + Kd_z * (-vel(3));
    U2 = Kp_phi * errores(2) + Ki_phi * integral_phi + Kd_phi * (-vel(4));
    U3 = Kp_theta * errores(3) + Ki_theta * integral_theta + Kd_theta * (-vel(5));
    U4 = Kp_psi * errores(4) + Ki_psi * integral_psi + Kd_psi * (-vel(6));
    
    % Dinámica traslacional
    acc_lin = [
        (cos(pos(4))*sin(pos(5))*cos(pos(6)) + sin(pos(4))*sin(pos(6))) * U1 / m;
        (cos(pos(4))*sin(pos(5))*sin(pos(6)) - sin(pos(4))*cos(pos(6))) * U1 / m;
        (cos(pos(4))*cos(pos(5)) * U1 / m) - g
    ];
    
    % Dinámica rotacional
    acc_ang = [
        (U2 + (Iy - Iz)*vel(5)*vel(6)) / Ix;
        (U3 + (Iz - Ix)*vel(4)*vel(6)) / Iy;
        (U4 + (Ix - Iy)*vel(4)*vel(5)) / Iz
    ];
    
    dXdt = [vel; acc_lin; acc_ang];
end