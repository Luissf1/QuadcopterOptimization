% --- Parámetros y condiciones iniciales (igual que tu código) ---
m = 1.0;          % Masa 
g = 9.81;         % Gravedad 
Ix = 0.1; Iy = 0.1; Iz = 0.2;  % Momentos de inercia 

x0 = [0; 0; 0; 0; 0; 0];       % Posición inicial [x, y, z, roll, pitch, yaw]
xdot0 = [0; 0; 0; 0; 0; 0];    % Velocidades iniciales

X0 = [x0; xdot0];              % Estado completo inicial

% Tiempo de simulación
tspan = [0 20];

% Deseados (mantener constantes para prueba ZN)
z_des = 1; phi_des = 0; theta_des = 0; psi_des = 0;

% --- PASO 1: Encontrar Ganancia Crítica Ku y Periodo de Oscilación Pu para cada canal ---
% Esto se hace incrementando Kp mientras Ki y Kd son 0, hasta que la salida oscile en amplitud constante.

% Función para encontrar Ku y Pu (búsqueda manual o automática)
% Aquí se muestra un ejemplo manual para canal 'z' (altitud). 
% Repite similar para phi, theta y psi.

% Valores iniciales para búsqueda
Ki_test = 0; 
Kd_test = 0;

% Rango de Kp a probar (incremental)
Kp_values = 0:1:50;  % Ajusta según tu sistema

Ku_z = NaN; Pu_z = NaN; % Inicializar

for Kp_test = Kp_values
    % Reiniciar variables globales de integral
    global integral_z integral_phi integral_theta integral_psi;
    integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;

    % Simular con Kp_test en z, Ki=0 y Kd=0
    [t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
        Kp_test, Ki_test, Kd_test, 0,0,0, 0,0,0, 0,0,0,...
        z_des, phi_des, theta_des, psi_des), tspan, X0);

    % Extraer señal de salida (altitud)
    z = X(:,3);

    % Revisar si la salida presenta oscilaciones sostenidas
    % Puedes usar análisis simple de oscilaciones:
    % - buscar cruces por cero en la diferencia z-z_des
    z_error = z - z_des;
    zero_crossings = find(diff(sign(z_error)));

    if length(zero_crossings) > 5  % Si tiene más de 5 cruces, puede ser oscilación sostenida
        % Calcular periodo Pu como promedio de diferencias entre cruces
        Pu_z = mean(diff(t(zero_crossings)));
        Ku_z = Kp_test;
        fprintf('Altitud: Ku = %.2f, Pu = %.2f\n', Ku_z, Pu_z);
        break;
    end
end

% Si quieres, repite el mismo proceso para phi, theta y psi (control de actitud)

% --- PASO 2: Calcular parámetros PID con fórmulas Ziegler-Nichols ---
% Fórmulas para controlador PID clásico ZN:
% Kp = 0.6 * Ku
% Ki = 2 * Kp / Pu
% Kd = Kp * Pu / 8

% Para altitud (z)
Kp_z = 0.6 * Ku_z;
Ki_z = 2 * Kp_z / Pu_z;
Kd_z = Kp_z * Pu_z / 8;

% Imprimir resultados
fprintf('Parámetros ZN para altitud (z): Kp=%.2f, Ki=%.2f, Kd=%.2f\n', Kp_z, Ki_z, Kd_z);

% Para actitud (roll, pitch, yaw) se recomienda hacer lo mismo (en este ejemplo uso valores aproximados)
Kp_phi = 0.6 * Ku_z;    Ki_phi = 2 * Kp_phi / Pu_z;    Kd_phi = Kp_phi * Pu_z / 8;
Kp_theta = Kp_phi;      Ki_theta = Ki_phi;             Kd_theta = Kd_phi;
Kp_psi = Kp_phi;        Ki_psi = Ki_phi;               Kd_psi = Kd_phi;

% --- PASO 3: Simular con parámetros PID obtenidos ---
global integral_z integral_phi integral_theta integral_psi;
integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;

[t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
    Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
    Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
    z_des, phi_des, theta_des, psi_des), tspan, X0);

% --- PASO 4: Graficar resultados ---
figure;
subplot(2,2,1);
plot(t, X(:,3), 'b', t, z_des*ones(size(t)), 'r--');
title('Altitud con PID Ziegler-Nichols');
xlabel('Tiempo (s)'); ylabel('Altitud (m)');
legend('Salida','Deseado'); grid on;

subplot(2,2,2);
plot(t, X(:,4), 'b', t, phi_des*ones(size(t)), 'r--');
title('Roll con PID Ziegler-Nichols');
xlabel('Tiempo (s)'); ylabel('Roll (rad)');
legend('Salida','Deseado'); grid on;

subplot(2,2,3);
plot(t, X(:,5), 'b', t, theta_des*ones(size(t)), 'r--');
title('Pitch con PID Ziegler-Nichols');
xlabel('Tiempo (s)'); ylabel('Pitch (rad)');
legend('Salida','Deseado'); grid on;

subplot(2,2,4);
plot(t, X(:,6), 'b', t, psi_des*ones(size(t)), 'r--');
title('Yaw con PID Ziegler-Nichols');
xlabel('Tiempo (s)'); ylabel('Yaw (rad)');
legend('Salida','Deseado'); grid on;

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
    
    % Actualizar integrales
    integral_z = integral_z + errores(1);
    integral_phi = integral_phi + errores(2);
    integral_theta = integral_theta + errores(3);
    integral_psi = integral_psi + errores(4);
    
    % Control PID
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
    
    % Vector derivadas
    dXdt = [vel; acc_lin; acc_ang];
end


