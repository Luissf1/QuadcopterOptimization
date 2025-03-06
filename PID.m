% Parámetros físicos
m = 1.0; % Masa (kg)
g = 9.81; % Gravedad (m/s^2)
Ix = 0.1; Iy = 0.1; Iz = 0.2; % Momentos de inercia (kg*m^2)

% Condiciones iniciales
x0 = [0; 0; 0; 0; 0; 0]; % [x, y, z, phi, theta, psi]
xdot0 = [0; 0; 0; 0; 0; 0]; % [dx, dy, dz, dphi, dtheta, dpsi]

% Ganchos PID para Z, Roll, Pitch, Yaw
Kp_z = 10; Ki_z = 0.1; Kd_z = 5;
Kp_phi = 5; Ki_phi = 0.01; Kd_phi = 1;
Kp_theta = 5; Ki_theta = 0.01; Kd_theta = 1;
Kp_psi = 5; Ki_psi = 0.01; Kd_psi = 1;

% Tiempo de simulación
tspan = [0 10];

% Estado inicial
X0 = [x0; xdot0];

% Variables para almacenar las integrales del error
global integral_z integral_phi integral_theta integral_psi;
integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;

% Resolver las ecuaciones diferenciales
[t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz, ...
    Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi, ...
    Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi), tspan, X0);

% Graficar la altitud
figure;
plot(t, X(:, 3));
xlabel('Tiempo (s)');
ylabel('Altitud (m)');
title('Altitud del Quadrotor');

% Función de dinámica del quadrotor
function dXdt = quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz, ...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi, ...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi)

    % Variables globales para las integrales del error
    global integral_z integral_phi integral_theta integral_psi;

    % Extraer estados
    x = X(1); y = X(2); z = X(3);
    phi = X(4); theta = X(5); psi = X(6);
    dx = X(7); dy = X(8); dz = X(9);
    dphi = X(10); dtheta = X(11); dpsi = X(12);

    % Estados deseados
    z_des = 1; % Altitud deseada
    phi_des = 0; % Roll deseado
    theta_des = 0; % Pitch deseado
    psi_des = 0; % Yaw deseado

    % Calcular errores
    error_z = z_des - z;
    error_phi = phi_des - phi;
    error_theta = theta_des - theta;
    error_psi = psi_des - psi;

    % Actualizar las integrales del error
    integral_z = integral_z + error_z;
    integral_phi = integral_phi + error_phi;
    integral_theta = integral_theta + error_theta;
    integral_psi = integral_psi + error_psi;

    % Control PID para Z
    U1 = Kp_z * error_z + Ki_z * integral_z + Kd_z * (0 - dz);

    % Control PID para Roll, Pitch, Yaw
    U2 = Kp_phi * error_phi + Ki_phi * integral_phi + Kd_phi * (0 - dphi);
    U3 = Kp_theta * error_theta + Ki_theta * integral_theta + Kd_theta * (0 - dtheta);
    U4 = Kp_psi * error_psi + Ki_psi * integral_psi + Kd_psi * (0 - dpsi);

    % Ecuaciones de movimiento
    ddx = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * U1 / m;
    ddy = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * U1 / m;
    ddz = (cos(phi)*cos(theta)) * U1 / m - g;

    ddphi = (U2 + (Iy - Iz)*dtheta*dpsi) / Ix;
    ddtheta = (U3 + (Iz - Ix)*dphi*dpsi) / Iy;
    ddpsi = (U4 + (Ix - Iy)*dphi*dtheta) / Iz;

    % Retornar derivadas
    dXdt = [dx; dy; dz; dphi; dtheta; dpsi; ddx; ddy; ddz; ddphi; ddtheta; ddpsi];
end