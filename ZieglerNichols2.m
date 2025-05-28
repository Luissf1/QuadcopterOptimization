
%% Script completo: Sintonización Ziegler-Nichols para cuadrotor (4 ejes)

clear; close all; clc;
format shortG;

%% Parámetros físicos del cuadrotor
m = 1.0;          % Masa (kg)
g = 9.81;         % Gravedad (m/s²)
Ix = 0.1; Iy = 0.1; Iz = 0.2;  % Momentos de inercia (kg·m²)

% Estado inicial: posición y velocidades
x0 = zeros(6,1);      % [x, y, z, phi, theta, psi]
v0 = zeros(6,1);      % Velocidades [dx, dy, dz, dphi, dtheta, dpsi]
X0 = [x0; v0];

% Referencias deseadas
refs = struct('altitude', 1, 'roll', 0, 'pitch', 0, 'yaw', 0);

% Tiempo de simulación para búsqueda de ganancia crítica (más largo para detectar oscilaciones)
tspan_find = [0 20];

% Parámetros Ziegler-Nichols (clásicos para PID)
ZN_table = struct(...
    'P',  [0.5, 0, 0], ...
    'PI', [0.45, 1/1.2, 0], ...
    'PID',[0.6, 2/1.2, 0.125]);

tipo_control = 'PID'; % Cambiar a 'P' o 'PI' si quieres otro tipo de controlador

%% Buscar Kc y Pu para cada eje
axes_names = {'altitude', 'roll', 'pitch', 'yaw'};
Kc = NaN(1,4);
Pu = NaN(1,4);

for i = 1:length(axes_names)
    axis_name = axes_names{i};
    fprintf('Buscando Kc y Pu para el eje %s...\n', axis_name);
    
    try
        [Kc(i), Pu(i)] = find_PID_ZN(axis_name, m, g, Ix, Iy, Iz, refs, tspan_find, X0);
        fprintf('Oscilación sostenida con Kc=%.3f, Pu=%.2f s\n', Kc(i), Pu(i));
    catch ME
        warning('No se encontró ganancia crítica para el eje %s: %s', axis_name, ME.message);
        Kc(i) = NaN;
        Pu(i) = NaN;
    end
end

%% Calcular parámetros PID con Ziegler-Nichols clásico
Kp = NaN(size(Kc));
Ki = NaN(size(Kc));
Kd = NaN(size(Kc));

for i=1:length(Kc)
    if ~isnan(Kc(i)) && ~isnan(Pu(i))
        Kp(i) = ZN_table.(tipo_control)(1)*Kc(i);
        if ZN_table.(tipo_control)(2) > 0
            Ki(i) = Kp(i)/(ZN_table.(tipo_control)(2)*Pu(i));
        else
            Ki(i) = 0;
        end
        if ZN_table.(tipo_control)(3) > 0
            Kd(i) = Kp(i)*ZN_table.(tipo_control)(3)*Pu(i);
        else
            Kd(i) = 0;
        end
    end
end

fprintf('\nParámetros PID calculados (Kp, Ki, Kd):\n');
for i=1:length(axes_names)
    fprintf('%s: Kp=%.3f, Ki=%.3f, Kd=%.3f\n', axes_names{i}, Kp(i), Ki(i), Kd(i));
end

%% Simular sistema completo con PID ZN
tspan_sim = [0 10];
[t, X] = ode45(@(t,X) quadrotor_dynamics(t,X,m,g,Ix,Iy,Iz,...
    Kp(1), Ki(1), Kd(1), ...
    Kp(2), Ki(2), Kd(2), ...
    Kp(3), Ki(3), Kd(3), ...
    Kp(4), Ki(4), Kd(4), ...
    refs.altitude, refs.roll, refs.pitch, refs.yaw), tspan_sim, X0);

% Gráfica resultados
figure('Name','Respuesta del Cuadrotor con PID Ziegler-Nichols','Position',[100 100 900 600]);

subplot(2,2,1);
plot(t, X(:,3),'b', t, refs.altitude*ones(size(t)),'r--','LineWidth',1.5);
title('Altitud'); xlabel('Tiempo (s)'); ylabel('Altura (m)');
legend('Salida','Referencia'); grid on;

subplot(2,2,2);
plot(t, X(:,4),'b', t, refs.roll*ones(size(t)),'r--','LineWidth',1.5);
title('Roll'); xlabel('Tiempo (s)'); ylabel('Ángulo (rad)');
grid on;

subplot(2,2,3);
plot(t, X(:,5),'b', t, refs.pitch*ones(size(t)),'r--','LineWidth',1.5);
title('Pitch'); xlabel('Tiempo (s)'); ylabel('Ángulo (rad)');
grid on;

subplot(2,2,4);
plot(t, X(:,6),'b', t, refs.yaw*ones(size(t)),'r--','LineWidth',1.5);
title('Yaw'); xlabel('Tiempo (s)'); ylabel('Ángulo (rad)');
grid on;

%% Función para buscar Kc y Pu por Ziegler-Nichols
function [Kc, Pu] = find_PID_ZN(axis, m, g, Ix, Iy, Iz, refs, tspan, X0)
    % Búsqueda adaptativa de ganancia crítica Kc para eje específico
    Kp_test = 0.1;   % Kp inicial
    Kp_max = 50;     % Kp máximo a probar
    Kp_step = 0.1;   % Incremento Kp

    Pu = NaN;
    last_oscillation_time = NaN;

    while Kp_test < Kp_max
        % Parámetros PID para prueba: solo proporcional con Kp_test
        Ki_test = 0;
        Kd_test = 0;

        % Simular el sistema en el eje solicitado
        [t, X] = ode45(@(t,X) quadrotor_dynamics_single_axis(t,X,m,g,Ix,Iy,Iz,axis,Kp_test,Ki_test,Kd_test,...
            refs.altitude, refs.roll, refs.pitch, refs.yaw), tspan, X0);

        % Extraer señal del eje
        y = get_axis_output(X, axis);

        % Detectar oscilaciones: cruces por cero del error respecto a referencia
        err = refs.(axis) - y;
        zero_crossings = find_zero_crossings(err);

        % Verificar si hay oscilación sostenida: al menos 6 cruces seguidos, amplitud > 1% referencia
        if length(zero_crossings) >= 6
            % Medir períodos entre cruces (usar cada 2 cruces para ciclo completo)
            periods = diff(t(zero_crossings));
            Pu_candidate = mean(periods);

            % Amplitud de oscilación para validación
            amp = max(y) - min(y);

            if amp > 0.01 * abs(refs.(axis)) % oscilación significativa
                Kc = Kp_test;
                Pu = Pu_candidate;
                return;
            end
        end

        Kp_test = Kp_test + Kp_step;
    end

    error('No se encontró ganancia crítica Kc para el eje %s en el rango de Kp probado.', axis);
end

%% Función dinámica simplificada para búsqueda de Kc (solo un eje)
function dXdt = quadrotor_dynamics_single_axis(t, X, m, g, Ix, Iy, Iz, axis, Kp, Ki, Kd,...
    z_des, phi_des, theta_des, psi_des)
    
    pos = X(1:6);
    vel = X(7:12);

    % Errores
    err = [z_des - pos(3); phi_des - pos(4); theta_des - pos(5); psi_des - pos(6)];

    % Integrales persistentes
    persistent integral_z integral_phi integral_theta integral_psi;
    if isempty(integral_z)
        integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;
    end
    
    % Actualizar integral solo para el eje que interesa
    switch axis
        case 'altitude'
            integral_z = integral_z + err(1);
        case 'roll'
            integral_phi = integral_phi + err(2);
        case 'pitch'
            integral_theta = integral_theta + err(3);
        case 'yaw'
            integral_psi = integral_psi + err(4);
    end

    % Control PID solo para eje actual
    switch axis
        case 'altitude'
            u = Kp*err(1) + Ki*integral_z + Kd*0; % Derivada ignorada para simplificar
            dz2 = u - g;
            dpos = vel(1:6);
            dvel = [0;0;dz2;0;0;0];
        case 'roll'
            u = Kp*err(2) + Ki*integral_phi + Kd*0;
            dpos = vel(1:6);
            dvel = [0;0;0; u/Ix; 0; 0];
        case 'pitch'
            u = Kp*err(3) + Ki*integral_theta + Kd*0;
            dpos = vel(1:6);
            dvel = [0;0;0;0; u/Iy; 0];
        case 'yaw'
            u = Kp*err(4) + Ki*integral_psi + Kd*0;
            dpos = vel(1:6);
            dvel = [0;0;0;0;0; u/Iz];
        otherwise
            error('Eje desconocido');
    end
    
    dXdt = [dpos; dvel];
end

%% Función dinámica completa para simular con PID en los 4 ejes
function dXdt = quadrotor_dynamics(t,X,m,g,Ix,Iy,Iz,...
    Kp_z, Ki_z, Kd_z,...
    Kp_phi, Ki_phi, Kd_phi,...
    Kp_theta, Ki_theta, Kd_theta,...
    Kp_psi, Ki_psi, Kd_psi,...
    z_des, phi_des, theta_des, psi_des)

    pos = X(1:6);
    vel = X(7:12);

    % Errores
    err = [z_des - pos(3); phi_des - pos(4); theta_des - pos(5); psi_des - pos(6)];

    % Integrales persistentes
    persistent integral_z integral_phi integral_theta integral_psi last_t;
    if isempty(integral_z)
        integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0; last_t = t;
    end
    
    dt = t - last_t;
    if dt < 0, dt = 0; end % para primeros pasos
    
    integral_z = integral_z + err(1)*dt;
    integral_phi = integral_phi + err(2)*dt;
    integral_theta = integral_theta + err(3)*dt;
    integral_psi = integral_psi + err(4)*dt;
    last_t = t;

    % Derivadas (velocidades angulares)
    % Aproximamos derivada de error con la velocidad actual para simplificación
    d_err = -vel(4:7); % phi_dot, theta_dot, psi_dot; z_dot no se usa para derivada en altura simple
    
    % Control PID para cada eje
    u_z = Kp_z*err(1) + Ki_z*integral_z + Kd_z*d_err(1);
    u_phi = Kp_phi*err(2) + Ki_phi*integral_phi + Kd_phi*d_err(2);
    u_theta = Kp_theta*err(3) + Ki_theta*integral_theta + Kd_theta*d_err(3);
    u_psi = Kp_psi*err(4) + Ki_psi*integral_psi + Kd_psi*d_err(4);

    % Dinámica simplificada:
    dpos = vel(1:6);
    dvel = zeros(6,1);
    % Altitud (eje z)
    dvel(3) = u_z - g;
    % Roll (phi)
    dvel(4) = u_phi / Ix;
    % Pitch (theta)
    dvel(5) = u_theta / Iy;
    % Yaw (psi)
    dvel(6) = u_psi / Iz;

    dXdt = [dpos; dvel];
end

%% Función para obtener salida de un eje en X
function y = get_axis_output(X, axis)
    switch axis
        case 'altitude'
            y = X(:,3);
        case 'roll'
            y = X(:,4);
        case 'pitch'
            y = X(:,5);
        case 'yaw'
            y = X(:,6);
        otherwise
            error('Eje desconocido');
    end
end

%% Función para detectar cruces por cero (cambios de signo)
function idx = find_zero_crossings(signal)
    idx = find(signal(1:end-1).*signal(2:end) < 0) + 1;
end
