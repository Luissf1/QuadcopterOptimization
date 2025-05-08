%% Código Completo para Simulación PID con Cálculo de Fitness
clc;
clear;
close all;

%% Parámetros del sistema
m = 1.0;          % Masa del cuadricóptero [kg]
g = 9.81;         % Aceleración gravitacional [m/s^2]
Ix = 0.1; Iy = 0.1; Iz = 0.2;  % Momentos de inercia [kg·m^2]

%% Condiciones iniciales
x0 = [0; 0; 0; 0; 0; 0];       % Posición inicial [x, y, z, ϕ, θ, ψ]
xdot0 = [0; 0; 0; 0; 0; 0];    % Velocidad inicial [dx, dy, dz, dϕ, dθ, dψ]
X0 = [x0; xdot0];              % Vector de estado inicial

%% Configuración de simulación
tspan = [0 10];   % Tiempo de simulación [s]

%% Valores deseados (configura según tu experimento)
z_des = 1;        % Altitud deseada [m]
phi_des = 0.2;    % Ángulo de roll deseado [rad]
theta_des = 0.1;  % Ángulo de pitch deseado [rad]
psi_des = 0;      % Ángulo de yaw deseado [rad]

%% Parámetros PID (valores iniciales)
% Estos son los valores que se mantienen constantes durante la simulación
Kp_z = 10; Ki_z = 0.1; Kd_z = 5;          % Control de altitud
Kp_phi = 5; Ki_phi = 0.01; Kd_phi = 1;    % Control de Roll
Kp_theta = 5; Ki_theta = 0.01; Kd_theta = 1; % Control de Pitch
Kp_psi = 5; Ki_psi = 0.01; Kd_psi = 1;    % Control de Yaw

%% Variables globales para integrales PID
global integral_z integral_phi integral_theta integral_psi;
integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;

%% Simulación del sistema
[t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
    Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
    Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
    z_des, phi_des, theta_des, psi_des), tspan, X0);

%% Cálculo del fitness (ITAE)
fitness = calcular_fitness(t, X, z_des, phi_des, theta_des, psi_des);

%% Mostrar resultados
disp('=== Valores finales de las ganancias PID ===');
disp(['Altitud: Kp=', num2str(Kp_z), ', Ki=', num2str(Ki_z), ', Kd=', num2str(Kd_z)]);
disp(['Roll:    Kp=', num2str(Kp_phi), ', Ki=', num2str(Ki_phi), ', Kd=', num2str(Kd_phi)]);
disp(['Pitch:   Kp=', num2str(Kp_theta), ', Ki=', num2str(Ki_theta), ', Kd=', num2str(Kd_theta)]);
disp(['Yaw:     Kp=', num2str(Kp_psi), ', Ki=', num2str(Ki_psi), ', Kd=', num2str(Kd_psi)]);
disp(' ');
disp(['Fitness calculado (ITAE): ', num2str(fitness)]);

%% Función de dinámica del cuadricóptero
function dXdt = quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
        z_des, phi_des, theta_des, psi_des)
    
    global integral_z integral_phi integral_theta integral_psi;
    
    % Extraer estados
    pos = X(1:6);       % [x, y, z, ϕ, θ, ψ]
    vel = X(7:12);      % [dx, dy, dz, dϕ, dθ, dψ]
    
    % Cálculo de errores
    errores = [z_des - pos(3);    % Error altitud
              phi_des - pos(4);   % Error Roll
              theta_des - pos(5); % Error Pitch
              psi_des - pos(6)];  % Error Yaw
    
    % Actualizar integrales (sin cambios)
    integral_z = integral_z + errores(1);
    integral_phi = integral_phi + errores(2);
    integral_theta = integral_theta + errores(3);
    integral_psi = integral_psi + errores(4);
    
    % Control PID (usando los valores constantes de Kp, Ki, Kd)
    U1 = Kp_z*errores(1) + Ki_z*integral_z + Kd_z*(-vel(3));
    U2 = Kp_phi*errores(2) + Ki_phi*integral_phi + Kd_phi*(-vel(4));
    U3 = Kp_theta*errores(3) + Ki_theta*integral_theta + Kd_theta*(-vel(5));
    U4 = Kp_psi*errores(4) + Ki_psi*integral_psi + Kd_psi*(-vel(6));
    
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

%% Función para calcular el fitness (ITAE)
function fitness = calcular_fitness(t, X, z_des, phi_des, theta_des, psi_des)
    % Extraer estados relevantes
    z = X(:,3);
    phi = X(:,4);
    theta = X(:,5);
    psi = X(:,6);
    
    % Crear vectores de referencia
    z_ref = z_des * ones(size(t));
    phi_ref = phi_des * ones(size(t));
    theta_ref = theta_des * ones(size(t));
    psi_ref = psi_des * ones(size(t));
    
    % Calcular errores
    error_z = z_ref - z;
    error_phi = phi_ref - phi;
    error_theta = theta_ref - theta;
    error_psi = psi_ref - psi;
    
    % Calcular ITAE (Integral of Time-weighted Absolute Error)
    itae_z = trapz(t, t.*abs(error_z));
    itae_phi = trapz(t, t.*abs(error_phi));
    itae_theta = trapz(t, t.*abs(error_theta));
    itae_psi = trapz(t, t.*abs(error_psi));
    
    % Fitness total
    fitness = itae_z + itae_phi + itae_theta + itae_psi;
end