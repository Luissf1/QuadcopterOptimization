%% Optimización y mostrar resultados
[global_best, convergence] = optimize_pid_with_pso();

% Mostrar mejores ganancias
disp('=== Mejores ganancias encontradas ===');
fprintf('Altura (z): Kp=%.4f, Ki=%.4f, Kd=%.4f\n', global_best.position(1:3));
fprintf('Roll (φ):   Kp=%.4f, Ki=%.4f, Kd=%.4f\n', global_best.position(4:6));
fprintf('Pitch (θ):  Kp=%.4f, Ki=%.4f, Kd=%.4f\n', global_best.position(7:9));
fprintf('Yaw (ψ):    Kp=%.4f, Ki=%.4f, Kd=%.4f\n', global_best.position(10:12));

% Graficar convergencia final
figure;
semilogy(convergence, 'b-o', 'LineWidth', 2);
xlabel('Iteración'); ylabel('Fitness (ITAE)');
title('Convergencia Final del PSO'); grid on;

%% Simular con las mejores ganancias encontradas
optimal_gains = global_best.position;
Kp_z = optimal_gains(1); Ki_z = optimal_gains(2); Kd_z = optimal_gains(3);
Kp_phi = optimal_gains(4); Ki_phi = optimal_gains(5); Kd_phi = optimal_gains(6);
Kp_theta = optimal_gains(7); Ki_theta = optimal_gains(8); Kd_theta = optimal_gains(9);
Kp_psi = optimal_gains(10); Ki_psi = optimal_gains(11); Kd_psi = optimal_gains(12);

% Parámetros del sistema
m = 1.0; g = 9.81; Ix = 0.1; Iy = 0.1; Iz = 0.2;
x0 = zeros(6,1); xdot0 = zeros(6,1); X0 = [x0; xdot0];
tspan = [0 10];
z_des = 5; phi_des = pi/4; theta_des = pi/4; psi_des = 0;

% Simular
[t, X] = ode45(@(t,X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
              Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
              Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
              z_des, phi_des, theta_des, psi_des), tspan, X0);

% Graficar resultados
figure;
subplot(2,2,1);
plot(t, X(:,3), 'b', t, z_des*ones(size(t)), 'r--');
title('Altitud (z)'); xlabel('Tiempo'); ylabel('m'); legend('Real', 'Deseado');

subplot(2,2,2);
plot(t, X(:,4), 'b', t, phi_des*ones(size(t)), 'r--');
title('Roll (φ)'); xlabel('Tiempo'); ylabel('rad');

subplot(2,2,3);
plot(t, X(:,5), 'b', t, theta_des*ones(size(t)), 'r--');
title('Pitch (θ)'); xlabel('Tiempo'); ylabel('rad');

subplot(2,2,4);
plot(t, X(:,6), 'b', t, psi_des*ones(size(t)), 'r--');
title('Yaw (ψ)'); xlabel('Tiempo'); ylabel('rad');

%% Algoritmo PSO para optimizar PID
function [global_best, B] = optimize_pid_with_pso()
    % Parámetros del PSO
    nVar = 12; % 4 controladores × 3 parámetros (Kp, Ki, Kd)
    % VarMin = [1.0  0.005  0.05 ...  % Límites para z (Kp, Ki, Kd)
    %           0.1  0.001  0.1 ...  % para phi
    %           0.1  0.001  0.1 ...  % para theta
    %           0.1  0.001  0.1];    % para psi
    % 
    % VarMax = [20   1     10 ...     % Límites para z
    %           10   0.1   5 ...     % para phi
    %           10   0.1   5 ...     % para theta
    %           10   0.1   5];       % para psi
      VarMin = [2.0  0.01  0.1 ... 
              0.1  0.001  0.1 ...  
              0.1  0.001  0.1 ...  
              0.1  0.001  0.1];    
          
      VarMax = [15   2.0   5 ...    
              10   0.1   2 ...    
              10   0.1   2 ...    
              10   0.1   2]; 
    
    MaxIter = 100;  % Iteraciones 
    nPop = 50;     % Población
    w = .7;         % Inercia inicial
    d = 0.97;      % Factor de decremento
    c1 = 1.7;      % Cognitivo
    c2 = 1.7;      % Social

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
        
        % Actualizar inercia (con límite inferior)
        w = max(w * d, 0.4);
        
        % Guardar mejor fitness
        B(iter) = global_best.fitness;
        fprintf('Iter %d: Mejor Fitness = %.4f\n', iter, B(iter));
        
        % Visualización
        if mod(iter,5) == 0
            figure(1);
            semilogy(B(1:iter), 'b-o', 'LineWidth', 1.5);
            xlabel('Iteración'); ylabel('Fitness (ITAE)');
            title('Convergencia del PSO'); grid on;
            drawnow;
        end
    end
end

%% Función Objetivo para evaluar el PID
function fitness = pid_objective_function(ganancias)
    % Parámetros fijos del cuadricóptero
    m = 1.0; g = 9.81; 
    Ix = 0.1; Iy = 0.1; Iz = 0.2;
    
    % Condiciones iniciales
    x0 = zeros(6,1); xdot0 = zeros(6,1);
    X0 = [x0; xdot0];
    tspan = [0 10];
    
    % Valores deseados 
    z_des = 5; 
    phi_des = pi/4; 
    theta_des = pi/4; 
    psi_des = 0;
    
    % Desempaquetar ganancias
    Kp_z = ganancias(1); Ki_z = ganancias(2); Kd_z = ganancias(3);
    Kp_phi = ganancias(4); Ki_phi = ganancias(5); Kd_phi = ganancias(6);
    Kp_theta = ganancias(7); Ki_theta = ganancias(8); Kd_theta = ganancias(9);
    Kp_psi = ganancias(10); Ki_psi = ganancias(11); Kd_psi = ganancias(12);
    
    % Simulación del sistema
    try
        [t, X] = ode45(@(t,X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
                          Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
                          Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
                          z_des, phi_des, theta_des, psi_des), tspan, X0);
        
        % Cálculo de errores (ITAE)
        error_z = abs(z_des - X(:,3));
        error_phi = abs(phi_des - X(:,4));
        error_theta = abs(theta_des - X(:,5));
        error_psi = abs(psi_des - X(:,6));
        
        % Penalización por sobreimpulso
        overshoot_penalty = 0;
        max_overshoot = max(0, max(X(:,3)) - z_des);
        if max_overshoot > 0
            overshoot_penalty = 10 * max_overshoot; % Más gradual
        end
         
        steady_state_error = mean(error_z(end-100:end)) + ...
                            mean(error_phi(end-100:end)) + ...
                            mean(error_theta(end-100:end)) + ...
                            mean(error_psi(end-100:end));

        % Fitness combinado
        fitness = trapz(t, t.*error_z) + trapz(t, t.*error_phi) + ...
                 trapz(t, t.*error_theta) + trapz(t, t.*error_psi) + ...
                 overshoot_penalty + 10*steady_state_error;
             
    catch
        fitness = 1e4 + norm(ganancias);
    end
end

%% Dinámica del Quadrotor
function dXdt = quadrotor_dynamics(~, X, m, g, Ix, Iy, Iz,...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
        z_des, phi_des, theta_des, psi_des)
    
    persistent integral_z integral_phi integral_theta integral_psi
    if isempty(integral_z)
        integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;
    end
    
    % Extraer estados
    pos = X(1:6);       % [x, y, z, ϕ, θ, ψ]
    vel = X(7:12);      % [dx, dy, dz, dϕ, dθ, dψ]
    
    % Cálculo de errores
    errores = [z_des - pos(3);    % Error altitud
              phi_des - pos(4);   % Error Roll
              theta_des - pos(5); % Error Pitch
              psi_des - pos(6)];  % Error Yaw
    
    % Actualizar integrales (con anti-windup básico)
    % Actualizar integrales (con anti-windup básico)
    max_int = 10;
    integral_z = max(min(integral_z + errores(1), max_int), -max_int);   
    integral_phi = max(min(integral_phi + errores(2), max_int), -max_int); 
    integral_theta = max(min(integral_theta + errores(3), max_int), -max_int); 
    integral_psi = max(min(integral_psi + errores(4), max_int), -max_int);
    
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
    
    % Vector de derivadas
    dXdt = [vel; acc_lin; acc_ang];
end

