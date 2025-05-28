%% Configuración inicial del sistema - Versión Optimizada
clear all; close all; clc;
format shortG;  % Formato de visualización con 4 decimales

% Parámetros físicos del cuadrotor
m = 1.0;          % Masa (kg)
g = 9.81;         % Gravedad (m/s²)
Ix = 0.1; Iy = 0.1; Iz = 0.2;  % Momentos de inercia (kg·m²)

% Condiciones iniciales
x0 = [0; 0; 0; 0; 0; 0];       % Posición [x, y, z, ϕ, θ, ψ]
xdot0 = [0; 0; 0; 0; 0; 0];    % Velocidades [dx, dy, dz, dϕ, dθ, dψ]

% Configuración de pruebas optimizada
num_pruebas = 30;  % Número reducido de pruebas
z_des = 1;         % Altitud deseada
phi_des = 0;       % Roll deseado
theta_des = 0;     % Pitch deseado
psi_des = 0;    % Yaw deseado
tspan = [0 5];     % Tiempo de simulación reducido

% Rangos para parámetros PID (optimizados)
rangos_pid = {
    % Eje       Kp_range        Ki_range        Kd_range
    'Altitud',  [8.0, 12.0],   [0.5, 1.0],     [4.0, 6.0];
    'Roll',     [1.5, 3.0],     [0.01, 0.05],   [0.1, 0.3];
    'Pitch',    [1.5, 3.0],     [0.01, 0.05],   [0.1, 0.3];
    'Yaw',      [0.5, 1.5],     [0.001, 0.01],  [0.05, 0.2];
};

% Pre-asignación de memoria para resultados
resultados = cell(num_pruebas, 1);

% Configuración de ODE para mejor rendimiento
opciones_ode = odeset('RelTol', 1e-3, 'AbsTol', 1e-4, 'Vectorized', 'on');

%% Bucle de pruebas optimizado
for prueba_actual = 1:num_pruebas
    fprintf('Prueba %d/%d...\n', prueba_actual, num_pruebas);
    
    % Generar parámetros PID aleatorios redondeados a 4 decimales
    params = cellfun(@(r) round(r(1) + (r(2)-r(1))*rand(), 4), rangos_pid(:,2:4), 'UniformOutput', false);
    
    % Extraer parámetros redondeados
    [Kp_z, Ki_z, Kd_z] = params{1,:};
    [Kp_phi, Ki_phi, Kd_phi] = params{2,:};
    [Kp_theta, Ki_theta, Kd_theta] = params{3,:};
    [Kp_psi, Ki_psi, Kd_psi] = params{4,:};
    
    % Estado inicial
    X0 = [x0; xdot0];
    
    % Variables globales para integrales PID (reset)
    global integral_z integral_phi integral_theta integral_psi;
    integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;
    
    % Simulación con opciones optimizadas
    [t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
        z_des, phi_des, theta_des, psi_des), tspan, X0, opciones_ode);
    
    % Vectores de referencia (pre-calculados)
    z_ref = z_des * ones(size(t));
    phi_ref = phi_des * ones(size(t));
    theta_ref = theta_des * ones(size(t));
    psi_ref = psi_des * ones(size(t));
    
    % Cálculo de métricas con redondeo a 4 decimales
    [IAE_z, ITSE_z, overshoot_z, settling_time_z] = calculate_metrics(t, X(:,3), z_ref);
    [IAE_phi, ~, overshoot_phi, settling_time_phi] = calculate_metrics(t, X(:,4), phi_ref);
    [IAE_theta, ~, overshoot_theta, settling_time_theta] = calculate_metrics(t, X(:,5), theta_ref);
    [IAE_psi, ~, overshoot_psi, settling_time_psi] = calculate_metrics(t, X(:,6), psi_ref);
    
    % Fitness combinado redondeado
    fitness = round(mean([IAE_z, IAE_phi, IAE_theta, IAE_psi]), 4);
    
    % Almacenar resultados en celda con valores redondeados
    resultados{prueba_actual} = table(...
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
    
    % Gráficos solo para la última prueba (opcional)
    if prueba_actual == num_pruebas
        plot_results(t, X, z_ref, phi_ref, theta_ref, psi_ref);
    end
end

%% Convertir resultados a tabla y guardar
resultados = vertcat(resultados{:});
writetable(resultados, 'Resultados_PID_Optimizados.xlsx');
fprintf('Simulación completada. Resultados guardados.\n');

%% Funciones optimizadas --------------------------------------------------

function [IAE, ITSE, overshoot, settling_time] = calculate_metrics(t, y, y_ref)
    err = y_ref - y;
    IAE = round(trapz(t, abs(err)), 4);
    ITSE = round(trapz(t, t.*err.^2), 4);
    
    y_final = y_ref(1);
    overshoot = round(max(0, (max(y) - y_final) / y_final * 100), 4);
    
    tol = 0.02 * abs(y_final);
    idx = find(abs(y - y_final) > tol, 1, 'last');
    if isempty(idx)
        settling_time = round(t(end), 4);
    else
        settling_time = round(t(min(idx+1, length(t))), 4);
    end
end

function plot_results(t, X, z_ref, phi_ref, theta_ref, psi_ref)
    figure('Name', 'Resultados Finales', 'Position', [100 100 900 600]);
    
    subplot(2,2,1);
    plot(t, X(:,3), 'b', t, z_ref, 'r--', 'LineWidth', 1.5);
    title('Altitud'); xlabel('Tiempo (s)'); ylabel('Altura (m)');
    legend('Obtenido', 'Deseado', 'Location', 'best');
    grid on;
    
    subplot(2,2,2);
    plot(t, X(:,4), 'b', t, phi_ref, 'r--', 'LineWidth', 1.5);
    title('Roll'); xlabel('Tiempo (s)'); ylabel('Ángulo (rad)');
    grid on;
    
    subplot(2,2,3);
    plot(t, X(:,5), 'b', t, theta_ref, 'r--', 'LineWidth', 1.5);
    title('Pitch'); xlabel('Tiempo (s)'); ylabel('Ángulo (rad)');
    grid on;
    
    subplot(2,2,4);
    plot(t, X(:,6), 'b', t, psi_ref, 'r--', 'LineWidth', 1.5);
    title('Yaw'); xlabel('Tiempo (s)'); ylabel('Ángulo (rad)');
    grid on;
end

function dXdt = quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
        z_des, phi_des, theta_des, psi_des)
    
    persistent integral_z integral_phi integral_theta integral_psi;
    
    % Inicializar integrales en la primera llamada
    if isempty(integral_z)
        integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;
    end
    
    % Estados actuales
    pos = X(1:6);       % [x, y, z, ϕ, θ, ψ]
    vel = X(7:12);      % Velocidades
    
    % Errores actuales
    errores = [
        z_des - pos(3);
        phi_des - pos(4);
        theta_des - pos(5);
        psi_des - pos(6)
    ];
    
    % Actualizar integrales (con limitación básica anti-windup)
    integral_z = integral_z + errores(1);
    integral_phi = integral_phi + errores(2);
    integral_theta = integral_theta + errores(3);
    integral_psi = integral_psi + errores(4);
    
    % Limitar integrales
    max_int = 10; % Límite arbitrario, ajustar según necesidad
    integral_z = sign(integral_z) * min(abs(integral_z), max_int);
    integral_phi = sign(integral_phi) * min(abs(integral_phi), max_int);
    integral_theta = sign(integral_theta) * min(abs(integral_theta), max_int);
    integral_psi = sign(integral_psi) * min(abs(integral_psi), max_int);
    
    % Ley de control PID
    U1 = Kp_z * errores(1) + Ki_z * integral_z + Kd_z * (-vel(3));
    U2 = Kp_phi * errores(2) + Ki_phi * integral_phi + Kd_phi * (-vel(4));
    U3 = Kp_theta * errores(3) + Ki_theta * integral_theta + Kd_theta * (-vel(5));
    U4 = Kp_psi * errores(4) + Ki_psi * integral_psi + Kd_psi * (-vel(6));
    
    % Pre-calcular términos trigonométricos comunes
    c_phi = cos(pos(4)); s_phi = sin(pos(4));
    c_theta = cos(pos(5)); s_theta = sin(pos(5));
    c_psi = cos(pos(6)); s_psi = sin(pos(6));
    
    % Dinámica traslacional optimizada
    rot_matrix = [
        (c_phi*s_theta*c_psi + s_phi*s_psi);
        (c_phi*s_theta*s_psi - s_phi*c_psi);
        (c_phi*c_theta)
    ];
    
    acc_lin = (U1/m) * rot_matrix;
    acc_lin(3) = acc_lin(3) - g;  % Añadir gravedad solo en z
    
    % Dinámica rotacional optimizada
    acc_ang = [
        (U2 + (Iy - Iz)*vel(5)*vel(6)) / Ix;
        (U3 + (Iz - Ix)*vel(4)*vel(6)) / Iy;
        (U4 + (Ix - Iy)*vel(4)*vel(5)) / Iz
    ];
    
    dXdt = [vel; acc_lin; acc_ang];
end