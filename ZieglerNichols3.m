% --- Parámetros y condiciones iniciales ---
m = 1.0; g = 9.81;
Ix = 0.1; Iy = 0.1; Iz = 0.2;

x0 = [0; 0; 0; 0; 0; 0];       % Posición inicial [x, y, z, roll, pitch, yaw]
xdot0 = [0; 0; 0; 0; 0; 0];    % Velocidades iniciales
X0 = [x0; xdot0];              % Estado completo

tspan = [0 20];

z_des = 1; phi_des = 0; theta_des = 0; psi_des = 0;

% --- Búsqueda de Ku y Pu ---
Ki_test = 0; Kd_test = 0;
Kp_values = 0:1:50;

Ku_z = NaN; Pu_z = NaN;

% Arreglos para graficar comportamiento del error
mse_values = [];
zero_crossings_count = [];
kp_record = [];

for Kp_test = Kp_values
    global integral_z integral_phi integral_theta integral_psi;
    integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;

    [t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
        Kp_test, Ki_test, Kd_test, 0,0,0, 0,0,0, 0,0,0,...
        z_des, phi_des, theta_des, psi_des), tspan, X0);

    z = X(:,3);
    z_error = z - z_des;
    kp_record(end+1) = Kp_test;

    mse = mean(z_error.^2);
    mse_values(end+1) = mse;

    zero_crossings = find(diff(sign(z_error)));
    zero_crossings_count(end+1) = length(zero_crossings);

    if length(zero_crossings) > 5
        Pu_z = mean(diff(t(zero_crossings)));
        Ku_z = Kp_test;
        fprintf('Altitud: Ku = %.2f, Pu = %.2f\n', Ku_z, Pu_z);
        break;
    end
end

% --- Visualización del comportamiento del error y oscilaciones ---
figure;
subplot(2,1,1);
plot(kp_record, mse_values, '-o');
xlabel('Kp probado'); ylabel('MSE'); title('Error cuadrático medio vs. Kp'); grid on;

subplot(2,1,2);
plot(kp_record, zero_crossings_count, '-o');
xlabel('Kp probado'); ylabel('Cruces por cero'); title('Oscilaciones vs. Kp'); grid on;

% --- Cálculo de parámetros PID Ziegler-Nichols ---
Kp_z = 0.6 * Ku_z;
Ki_z = 2 * Kp_z / Pu_z;
Kd_z = Kp_z * Pu_z / 8;

fprintf('Parámetros ZN para altitud (z): Kp=%.2f, Ki=%.2f, Kd=%.2f\n', Kp_z, Ki_z, Kd_z);

Kp_phi = Kp_z; Ki_phi = Ki_z; Kd_phi = Kd_z;
Kp_theta = Kp_z; Ki_theta = Ki_z; Kd_theta = Kd_z;
Kp_psi = Kp_z; Ki_psi = Ki_z; Kd_psi = Kd_z;

% --- Simulación final con PID ---
global integral_z integral_phi integral_theta integral_psi;
integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;

[t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
    Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
    Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
    z_des, phi_des, theta_des, psi_des), tspan, X0);

% --- Gráfica de resultados ---
figure;
subplot(2,2,1);
plot(t, X(:,3), 'b', t, z_des*ones(size(t)), 'r--');
title('Altitud con PID ZN'); xlabel('Tiempo'); ylabel('z (m)'); legend('Salida','Deseado'); grid on;

subplot(2,2,2);
plot(t, X(:,4), 'b', t, phi_des*ones(size(t)), 'r--');
title('Roll'); xlabel('Tiempo'); ylabel('ϕ (rad)'); legend('Salida','Deseado'); grid on;

subplot(2,2,3);
plot(t, X(:,5), 'b', t, theta_des*ones(size(t)), 'r--');
title('Pitch'); xlabel('Tiempo'); ylabel('θ (rad)'); legend('Salida','Deseado'); grid on;

subplot(2,2,4);
plot(t, X(:,6), 'b', t, psi_des*ones(size(t)), 'r--');
title('Yaw'); xlabel('Tiempo'); ylabel('ψ (rad)'); legend('Salida','Deseado'); grid on;


% --- Función de dinámica del dron ---
function dXdt = quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
        z_des, phi_des, theta_des, psi_des)

    global integral_z integral_phi integral_theta integral_psi;
    
    pos = X(1:6);       % [x, y, z, roll, pitch, yaw]
    vel = X(7:12);      % [dx, dy, dz, dϕ, dθ, dψ]
    
    errores = [z_des - pos(3);
               phi_des - pos(4);
               theta_des - pos(5);
               psi_des - pos(6)];
    
    integral_z = integral_z + errores(1);
    integral_phi = integral_phi + errores(2);
    integral_theta = integral_theta + errores(3);
    integral_psi = integral_psi + errores(4);
    
    U1 = Kp_z*errores(1) + Ki_z*integral_z + Kd_z*(-vel(3));
    U2 = Kp_phi*errores(2) + Ki_phi*integral_phi + Kd_phi*(-vel(4));
    U3 = Kp_theta*errores(3) + Ki_theta*integral_theta + Kd_theta*(-vel(5));
    U4 = Kp_psi*errores(4) + Ki_psi*integral_psi + Kd_psi*(-vel(6));
    
    acc_lin = [...
        (cos(pos(4))*sin(pos(5))*cos(pos(6)) + sin(pos(4))*sin(pos(6)))*U1/m;
        (cos(pos(4))*sin(pos(5))*sin(pos(6)) - sin(pos(4))*cos(pos(6)))*U1/m;
        (cos(pos(4))*cos(pos(5))*U1/m) - g];
    
    acc_ang = [...
        (U2 + (Iy - Iz)*vel(5)*vel(6))/Ix;
        (U3 + (Iz - Ix)*vel(4)*vel(6))/Iy;
        (U4 + (Ix - Iy)*vel(4)*vel(5))/Iz];
    
    dXdt = [vel; acc_lin; acc_ang];
end
