close all; clear; clc;

%% 1. PARÁMETROS DEL CUADRICÓPTERO (FÁCIL DE MODIFICAR)
% Definimos los parámetros físicos del drone
m = 1.0;       % Masa (kg)
g = 9.81;      % Gravedad (m/s²)
Ix = 0.1;      % Momento de inercia en x (kg·m²)
Iy = 0.1;      % Momento de inercia en y (kg·m²)
Iz = 0.2;      % Momento de inercia en z (kg·m²)

% Valores deseados (a donde queremos que llegue el drone)
z_des = 1;          % Altura deseada (m)
phi_des = 0;        % Ángulo de roll deseado (rad)
theta_des = 0;      % Ángulo de pitch deseado (rad)
psi_des = pi/4;     % Ángulo de yaw deseado (rad)

%% 2. CONFIGURACIÓN DEL RUIDO (FACTORES DEL MUNDO REAL)
% Estos parámetros simulan imperfecciones del mundo real
noise_settings.sensor = 0.02;   % 2% de ruido en sensores
noise_settings.wind = 0.5;      % Fuerza del viento (N)
noise_settings.actuator = 0.03; % 3% de ruido en motores
noise_settings.wind_change = 0.1; % Cambio en dirección del viento

%% 3. OPTIMIZACIÓN DE PARÁMETROS PID (USANDO PSO SIMPLIFICADO)
% Buscamos los mejores valores para Kp, Ki, Kd de cada controlador
fprintf('Optimizando parámetros PID...\n');
[best_gains, convergence] = simple_pso_optimizer(m, g, Ix, Iy, Iz, noise_settings, ...
                                z_des, phi_des, theta_des, psi_des);

% Mostramos los mejores valores encontrados
disp('=== Mejores parámetros PID encontrados ===');
fprintf('Altura (z): Kp=%.3f, Ki=%.3f, Kd=%.3f\n', best_gains(1:3));
fprintf('Roll (φ):   Kp=%.3f, Ki=%.3f, Kd=%.3f\n', best_gains(4:6));
fprintf('Pitch (θ):  Kp=%.3f, Ki=%.3f, Kd=%.3f\n', best_gains(7:9));
fprintf('Yaw (ψ):    Kp=%.3f, Ki=%.3f, Kd=%.3f\n', best_gains(10:12));

% Graficamos cómo mejoró el sistema durante la optimización
figure;
plot(convergence, 'b-o', 'LineWidth', 1.5);
xlabel('Iteración'); ylabel('Performance (menor es mejor)');
title('Mejoría durante la Optimización'); grid on;

%% 4. SIMULACIÓN FINAL CON RUIDO
fprintf('\nSimulando con los mejores parámetros...\n');

% Condiciones iniciales (drone en reposo en el origen)
initial_pos = zeros(6,1);       % [x, y, z, ϕ, θ, ψ]
initial_vel = zeros(6,1);       % [dx, dy, dz, dϕ, dθ, dψ]
X0 = [initial_pos; initial_vel]; % Estado inicial combinado
initial_pos(3) = 0.1;           % Pequeña altura inicial

% Tiempo de simulación (0 a 10 segundos)
tspan = [0 10];

% Opciones para la simulación (solver ode45)
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-5);

% Llamamos al solver de ecuaciones diferenciales
[t, X] = ode45(@(t,X) drone_dynamics_with_noise(t, X, m, g, Ix, Iy, Iz, ...
              best_gains, z_des, phi_des, theta_des, psi_des, ...
              noise_settings), tspan, X0, options);

%% 5. VISUALIZACIÓN DE RESULTADOS
figure('Name', 'Desempeño del Drone con Ruido');

% Altura (z)
subplot(2,2,1);
plot(t, X(:,3), 'b', t, z_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
title('Control de Altura');
xlabel('Tiempo (s)'); ylabel('Altura (m)');
legend('Real', 'Deseado'); grid on;

% Roll (ϕ)
subplot(2,2,2);
plot(t, X(:,4), 'b', t, phi_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
title('Control de Roll (ϕ)');
xlabel('Tiempo (s)'); ylabel('Ángulo (rad)'); grid on;

% Pitch (θ)
subplot(2,2,3);
plot(t, X(:,5), 'b', t, theta_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
title('Control de Pitch (θ)');
xlabel('Tiempo (s)'); ylabel('Ángulo (rad)'); grid on;

% Yaw (ψ)
subplot(2,2,4);
plot(t, X(:,6), 'b', t, psi_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
title('Control de Yaw (ψ)');
xlabel('Tiempo (s)'); ylabel('Ángulo (rad)'); grid on;

%% FUNCIÓN PRINCIPAL DE DINÁMICA CON RUIDO
function dXdt = drone_dynamics_with_noise(t, X, m, g, Ix, Iy, Iz, gains, ...
                                       z_des, phi_des, theta_des, psi_des, ...
                                       noise)
    persistent integral_errors last_errors wind_angle
    
    % Inicializamos variables persistentes (guardan valores entre llamadas)
    if isempty(integral_errors)
        integral_errors = zeros(4,1);  % Para z, ϕ, θ, ψ
        last_errors = zeros(4,1);
        wind_angle = 0;                % Dirección inicial del viento
    end
    
    % Extraemos posición y velocidad del estado actual
    pos = X(1:6);   % [x, y, z, ϕ, θ, ψ]
    vel = X(7:12);  % [dx, dy, dz, dϕ, dθ, dψ]
    
    %% A. SIMULACIÓN DE RUIDO EN SENSORES
    % Añadimos ruido a las mediciones que usaría el controlador
    measured_pos = pos + noise.sensor * randn(size(pos));
    measured_vel = vel + noise.sensor * randn(size(vel));
    
    %% B. SIMULACIÓN DE VIENTO (PERTURBACIÓN EXTERNA)
    % El viento cambia de dirección gradualmente
    wind_angle = wind_angle + noise.wind_change * randn;
    wind_force = noise.wind * [cos(wind_angle); sin(wind_angle); 0];
    
    %% C. CÁLCULO DE ERRORES (diferencia entre deseado y medido)
    errors = [z_des - measured_pos(3);
              phi_des - measured_pos(4);
              theta_des - measured_pos(5);
              psi_des - measured_pos(6)];
    
    %% D. CONTROL PID (con protección contra valores extremos)
    % Calculamos dt aproximado (para integral y derivada)
    if t == 0
        dt = 0.01;
    else
        dt = t - (t - 0.01); % Aproximación simple
    end
    
    % Actualizamos términos integrales (con límites)
    max_integral = 5;
    integral_errors = integral_errors + errors * dt;
    integral_errors = max(min(integral_errors, max_integral), -max_integral);
    
    % Calculamos derivadas (cambio en el error)
    derivatives = (errors - last_errors) / dt;
    
    % Desempaquetamos las ganancias PID
    Kp = gains(1:3:end);
    Ki = gains(2:3:end);
    Kd = gains(3:3:end);
    
    % Calculamos las acciones de control
    U1 = Kp(1)*errors(1) + Ki(1)*integral_errors(1) + Kd(1)*derivatives(1);
    U2 = Kp(2)*errors(2) + Ki(2)*integral_errors(2) + Kd(2)*derivatives(2);
    U3 = Kp(3)*errors(3) + Ki(3)*integral_errors(3) + Kd(3)*derivatives(3);
    U4 = Kp(4)*errors(4) + Ki(4)*integral_errors(4) + Kd(4)*derivatives(4);
    
    % Añadimos ruido a los actuadores (motores no perfectos)
    U1 = U1 * (1 + noise.actuator * randn);
    U2 = U2 * (1 + noise.actuator * randn);
    U3 = U3 * (1 + noise.actuator * randn);
    U4 = U4 * (1 + noise.actuator * randn);
    
    % Limitamos las acciones de control (físicamente realizable)
    U1 = max(min(U1, 20), 0);    % Empuje entre 0 y 20 N
    U2 = max(min(U2, 5), -5);    % Torques entre -5 y 5 Nm
    U3 = max(min(U3, 5), -5);
    U4 = max(min(U4, 5), -5);
    
    %% E. ECUACIONES DINÁMICAS DEL DRONE
    % Ángulos actuales (sin ruido, son los reales)
    phi = pos(4); theta = pos(5); psi = pos(6);
    
    % Matriz de rotación (transforma fuerzas del cuerpo al mundo)
    R = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
         cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
         -sin(theta)         sin(phi)*cos(theta)                            cos(phi)*cos(theta)];
    
    % Aceleración lineal (incluye gravedad y viento)
    acceleration = R * [0; 0; U1/m] - [0; 0; g] + wind_force/m;
    
    % Aceleración angular
    angular_acc = [(U2 + (Iy-Iz)*vel(5)*vel(6))/Ix;
                  (U3 + (Iz-Ix)*vel(4)*vel(6))/Iy;
                  (U4 + (Ix-Iy)*vel(4)*vel(5))/Iz];
    
    % Guardamos errores para la próxima iteración
    last_errors = errors;
    
    % Combinamos las derivadas para retornar
    dXdt = [vel; acceleration; angular_acc];
end

%% OPTIMIZADOR PSO SIMPLIFICADO
function [best_gains, convergence] = simple_pso_optimizer(m, g, Ix, Iy, Iz, noise, ...
                                                       z_des, phi_des, theta_des, psi_des)
    % Configuración básica del PSO
    n_particles = 20;   % Número de soluciones candidatas
    max_iter = 30;      % Número de iteraciones
    n_params = 12;      % 4 controladores × 3 parámetros (Kp, Ki, Kd)
    
    % Límites para los parámetros PID
    lower_bounds = [1.0  0.01  0.1  0.1  0.001  0.1  0.1  0.001  0.1  0.1  0.001  0.1];
    upper_bounds = [15   1.0   3.0  3.0  0.1    2.0  3.0  0.1    2.0  3.0  0.1    2.0];
    
    % Inicialización de partículas
    particles = rand(n_particles, n_params) .* (upper_bounds - lower_bounds) + lower_bounds;
    velocities = zeros(n_particles, n_params);
    best_particle_pos = particles;
    best_particle_scores = inf(n_particles, 1);
    
    % Mejor global
    global_best_pos = particles(1,:);
    global_best_score = inf;
    convergence = zeros(max_iter, 1);
    
    % Parámetros PSO
    inertia = 0.7;
    cognitive_weight = 1.5;
    social_weight = 1.5;
    
    % Bucle principal de optimización
    for iter = 1:max_iter
        for i = 1:n_particles
            % Evaluamos esta solución candidata
            current_score = evaluate_pid_performance(particles(i,:), m, g, Ix, Iy, Iz, ...
                                                  noise, z_des, phi_des, theta_des, psi_des);
            
            % Actualizamos mejor local
            if current_score < best_particle_scores(i)
                best_particle_pos(i,:) = particles(i,:);
                best_particle_scores(i) = current_score;
                
                % Actualizamos mejor global
                if current_score < global_best_score
                    global_best_pos = particles(i,:);
                    global_best_score = current_score;
                end
            end
            
            % Actualizamos velocidad y posición
            r1 = rand(1, n_params);
            r2 = rand(1, n_params);
            
            velocities(i,:) = inertia * velocities(i,:) + ...
                             cognitive_weight * r1 .* (best_particle_pos(i,:) - particles(i,:)) + ...
                             social_weight * r2 .* (global_best_pos - particles(i,:));
            
            particles(i,:) = particles(i,:) + velocities(i,:);
            
            % Aplicamos límites
            particles(i,:) = max(particles(i,:), lower_bounds);
            particles(i,:) = min(particles(i,:), upper_bounds);
        end
        
        % Guardamos progreso
        convergence(iter) = global_best_score;
        fprintf('Iteración %d: Mejor puntuación = %.2f\n', iter, global_best_score);
        
        % Reducimos inercia (búsqueda más local con el tiempo)
        inertia = inertia * 0.95;
    end
    
    best_gains = global_best_pos;
end

%% FUNCIÓN PARA EVALUAR EL DESEMPEÑO DE UN CONJUNTO PID
function score = evaluate_pid_performance(gains, m, g, Ix, Iy, Iz, noise, ...
                                       z_des, phi_des, theta_des, psi_des)
    % Configuración de simulación
    initial_pos = zeros(6,1);
    initial_vel = zeros(6,1);
    X0 = [initial_pos; initial_vel];
    initial_pos(3) = 0.1; % Pequeña altura inicial
    tspan = [0 10];
    
    % Simulamos el sistema con estos parámetros PID
    try
        [t, X] = ode45(@(t,X) drone_dynamics_with_noise(t, X, m, g, Ix, Iy, Iz, ...
                      gains, z_des, phi_des, theta_des, psi_des, noise), tspan, X0);
        
        % Calculamos métricas de desempeño
        z = X(:,3);
        phi = X(:,4);
        theta = X(:,5);
        psi = X(:,6);
        
        % Error cuadrático medio (penaliza grandes errores)
        mse_z = mean((z - z_des).^2);
        mse_phi = mean((phi - phi_des).^2);
        mse_theta = mean((theta - theta_des).^2);
        mse_psi = mean((psi - psi_des).^2);
        
        % Tiempo de establecimiento (cuándo se acerca al valor deseado)
        settling_threshold = 0.05; % 5% de margen
        settled_z = find(abs(z - z_des) < settling_threshold, 1);
        settled_psi = find(abs(psi - psi_des) < settling_threshold, 1);
        
        if isempty(settled_z), settled_z = length(t); end
        if isempty(settled_psi), settled_psi = length(t); end
        
        % Puntuación combinada (menor es mejor)
        score = mse_z + mse_phi + mse_theta + mse_psi + ...
                0.1*settled_z + 0.1*settled_psi;
        
    catch
        % Si la simulación falla, asignamos una mala puntuación
        score = 1e6;
    end
end