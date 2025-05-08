% Parámetros 
m = 1.0;          % Masa 
g = 9.81;         % Gravedad 
Ix = 0.1; Iy = 0.1; Iz = 0.2;  % Momentos de inercia 

% Condiciones iniciales
x0 = [0; 0; 0; 0; 0; 0];       % [x, y, z, Roll, Pitch, Yaw]
xdot0 = [0; 0; 0; 0; 0; 0];    % Velocidades iniciales [dx, dy, dz, dRoll, dPitch, dYaw]

% PID
Kp_z = 10; Ki_z = 0.1; Kd_z = 5;          % Control de altitud
Kp_phi = 5; Ki_phi = 0.01; Kd_phi = 1;    % Control de Roll
Kp_theta = 5; Ki_theta = 0.01; Kd_theta = 1; % Control de Pitch
Kp_psi = 5; Ki_psi = 0.01; Kd_psi = 1;    % Control de Yaw

z_des = 1;          % Altitud deseada
phi_des = 0.2;     % Roll deseado
theta_des = 0.1;   % Pitch deseado
psi_des = 0;        % Yaw deseado

% Tiempo de simulación
tspan = [0 10];

% Estado inicial
X0 = [x0; xdot0];

% Variables globales para integrales PID
global integral_z integral_phi integral_theta integral_psi;
integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;

% Resolver ecuaciones diferenciales (con valores deseados como parámetros)
[t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
    Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
    Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
    z_des, phi_des, theta_des, psi_des), tspan, X0);

% Crear vectores de valores deseados
z_des_vector = z_des * ones(size(t));
phi_des_vector = phi_des * ones(size(t));
theta_des_vector = theta_des * ones(size(t));
psi_des_vector = psi_des * ones(size(t));

% Graficar resultados (comparación deseado vs obtenido)
figure;
subplot(2,2,1);
plot(t, X(:,3), 'b', t, z_des_vector, 'r--', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Altitud (m)'); title('Altitud');
legend('Obtenida', 'Deseada'); grid on;

subplot(2,2,2);
plot(t, X(:,4), 'b', t, phi_des_vector, 'r--', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Roll (rad)'); title('Roll');
legend('Obtenido', 'Deseado'); grid on;

subplot(2,2,3);
plot(t, X(:,5), 'b', t, theta_des_vector, 'r--', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Pitch (rad)'); title('Pitch');
legend('Obtenido', 'Deseado'); grid on;

subplot(2,2,4);
plot(t, X(:,6), 'b', t, psi_des_vector, 'r--', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Yaw (rad)'); title('Yaw');
legend('Obtenido', 'Deseado'); grid on;

% Función de dinámica actualizada
function dXdt = quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
        z_des, phi_des, theta_des, psi_des) % <-- Parámetros añadidos

    global integral_z integral_phi integral_theta integral_psi;
    
    % Extraer estados
    pos = X(1:6);       % [x, y, z, ϕ, θ, ψ]
    vel = X(7:12);      % [dx, dy, dz, dϕ, dθ, dψ]
    
    % Cálculo de errores (usa los parámetros z_des, phi_des, etc.)
    errores = [z_des - pos(3);    % Error altitud
              phi_des - pos(4);   % Error Roll
              theta_des - pos(5); % Error Pitch
              psi_des - pos(6)];  % Error Yaw
    
    % Actualizar integrales
    integral_z = integral_z + errores(1);
    integral_phi = integral_phi + errores(2);
    integral_theta = integral_theta + errores(3);
    integral_psi = integral_psi + errores(4);
    
    % Control PID (usando los errores calculados)
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