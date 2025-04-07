% Parámetros del quadrotor
m = 1.0;           % Masa [kg]
g = 9.81;          % Gravedad [m/s²]
Ix = 0.1; Iy = 0.1; Iz = 0.2;  % Momentos de inercia [kg·m²]

% Valores deseados
z_des = 5;         % Altura deseada [m]
phi_des = pi/4;    % Ángulo roll deseado [rad]
theta_des = pi/4;  % Ángulo pitch deseado [rad]
psi_des = 0;       % Ángulo yaw deseado [rad]

% Tiempo de simulación
tspan = [0 10];    % Tiempo inicial y final [s]

%% ================= OPTIMIZACIÓN PSO MEJORADA =================
[mejores_ganancias, convergencia] = optimizar_pid();

% Mostrar resultados
mostrar_resultados(mejores_ganancias, convergencia);

%% ================= SIMULACIÓN CON GANANCIAS ÓPTIMAS =================
simular_con_ganancias(mejores_ganancias, m, g, [Ix, Iy, Iz], ...
                     [z_des, phi_des, theta_des, psi_des], tspan);

%% ================= FUNCIONES AUXILIARES =================
function mostrar_resultados(ganancias, convergencia)
    % Mostrar ganancias PID óptimas
    disp('=== Mejores ganancias PID encontradas ===');
    fprintf('Altura (z):   Kp = %6.3f, Ki = %6.3f, Kd = %6.3f\n', ganancias(1:3));
    fprintf('Roll (φ):     Kp = %6.3f, Ki = %6.3f, Kd = %6.3f\n', ganancias(4:6));
    fprintf('Pitch (θ):    Kp = %6.3f, Ki = %6.3f, Kd = %6.3f\n', ganancias(7:9));
    fprintf('Yaw (ψ):      Kp = %6.3f, Ki = %6.3f, Kd = %6.3f\n', ganancias(10:12));
    
    % Graficar convergencia en escala semilogarítmica
    figure;
    semilogy(convergencia, 'b-o', 'LineWidth', 2);
    xlabel('Iteración'); ylabel('Fitness (ITAE)');
    title('Convergencia del PSO'); grid on;
end

function simular_con_ganancias(ganancias, m, g, I, deseado, tspan)
    % Extraer parámetros
    Ix = I(1); Iy = I(2); Iz = I(3);
    z_des = deseado(1); phi_des = deseado(2);
    theta_des = deseado(3); psi_des = deseado(4);
    
    % Condiciones iniciales
    X0 = zeros(12,1);  % [posiciones; velocidades]
    
    % Simular sistema con ODE45
    [t, X] = ode45(@(t,X) dinamica_quadrotor(t, X, m, g, Ix, Iy, Iz, ...
                    ganancias, deseado), tspan, X0);
    
    % Graficar resultados
    figure;
    
    % Altura (z)
    subplot(2,2,1);
    plot(t, X(:,3), 'b', 'LineWidth', 1.5), hold on;
    plot(t, z_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
    title('Altura (z)'); xlabel('Tiempo (s)'); ylabel('m');
    legend('Real', 'Deseado', 'Location', 'best');
    grid on;
    
    % Roll (φ)
    subplot(2,2,2);
    plot(t, X(:,4), 'b', 'LineWidth', 1.5), hold on;
    plot(t, phi_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
    title('Roll (φ)'); xlabel('Tiempo (s)'); ylabel('rad');
    grid on;
    
    % Pitch (θ)
    subplot(2,2,3);
    plot(t, X(:,5), 'b', 'LineWidth', 1.5), hold on;
    plot(t, theta_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
    title('Pitch (θ)'); xlabel('Tiempo (s)'); ylabel('rad');
    grid on;
    
    % Yaw (ψ)
    subplot(2,2,4);
    plot(t, X(:,6), 'b', 'LineWidth', 1.5), hold on;
    plot(t, psi_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
    title('Yaw (ψ)'); xlabel('Tiempo (s)'); ylabel('rad');
    grid on;
end

%% ================= ALGORITMO PSO MEJORADO =================
function [mejor_ganancia, convergencia] = optimizar_pid()
    % Configuración del PSO
    n_variables = 12;  % 4 controladores × 3 parámetros (Kp, Ki, Kd)
    iteraciones = 100; % Número de iteraciones
    poblacion = 50;    % Tamaño de la población
    
    % Límites para los parámetros PID (más ajustados)
    limites_min = [2.0  0.01  0.1 ...    % Altura (z)
                  0.1  0.001  0.1 ...    % Roll (φ)
                  0.1  0.001  0.1 ...    % Pitch (θ)
                  0.1  0.001  0.1];      % Yaw (ψ)
                  
    limites_max = [15   2.0   5 ...       % Altura (z)
                  10   0.1   2 ...       % Roll (φ)
                  10   0.1   2 ...       % Pitch (θ)
                  10   0.1   2];         % Yaw (ψ)
    
    % Parámetros PSO
    inercia = 0.7;      % Peso de la velocidad anterior
    c1 = 1.7; c2 = 1.7; % Factores cognitivo y social
    factor_decremento = 0.97; % Factor de reducción de inercia
    
    % Inicialización
    mejor_global.posicion = [];
    mejor_global.fitness = inf;
    convergencia = zeros(iteraciones, 1);
    
    % Crear población inicial
    particulas = struct('posicion', [], 'velocidad', [], 'fitness', [], 'mejor_local', []);
    for i = 1:poblacion
        particulas(i).posicion = limites_min + rand(1,n_variables).*(limites_max-limites_min);
        particulas(i).velocidad = zeros(1,n_variables);
        particulas(i).fitness = evaluar_pid(particulas(i).posicion);
        particulas(i).mejor_local.posicion = particulas(i).posicion;
        particulas(i).mejor_local.fitness = particulas(i).fitness;
        
        % Actualizar mejor global
        if particulas(i).fitness < mejor_global.fitness
            mejor_global = particulas(i).mejor_local;
        end
    end
    
    % Bucle principal de optimización
    for iter = 1:iteraciones
        for i = 1:poblacion
            % Actualizar velocidad
            r1 = rand(1,n_variables);
            r2 = rand(1,n_variables);
            
            particulas(i).velocidad = inercia * particulas(i).velocidad + ...
                c1 * r1 .* (particulas(i).mejor_local.posicion - particulas(i).posicion) + ...
                c2 * r2 .* (mejor_global.posicion - particulas(i).posicion);
            
            % Actualizar posición
            particulas(i).posicion = particulas(i).posicion + particulas(i).velocidad;
            
            % Aplicar límites
            particulas(i).posicion = max(particulas(i).posicion, limites_min);
            particulas(i).posicion = min(particulas(i).posicion, limites_max);
            
            % Evaluar nueva posición
            particulas(i).fitness = evaluar_pid(particulas(i).posicion);
            
            % Actualizar mejor local
            if particulas(i).fitness < particulas(i).mejor_local.fitness
                particulas(i).mejor_local.posicion = particulas(i).posicion;
                particulas(i).mejor_local.fitness = particulas(i).fitness;
                
                % Actualizar mejor global
                if particulas(i).fitness < mejor_global.fitness
                    mejor_global = particulas(i).mejor_local;
                end
            end
        end
        
        % Reducir inercia
        inercia = max(inercia * factor_decremento, 0.4);
        
        % Guardar mejor fitness
        convergencia(iter) = mejor_global.fitness;
        fprintf('Iteración %3d: Mejor fitness = %.4f\n', iter, convergencia(iter));
    end
    
    mejor_ganancia = mejor_global.posicion;
end

%% ================= FUNCIÓN DE EVALUACIÓN MEJORADA =================
function fitness = evaluar_pid(ganancias)
    % Parámetros fijos
    m = 1.0; g = 9.81; 
    Ix = 0.1; Iy = 0.1; Iz = 0.2;
    
    % Condiciones iniciales
    X0 = zeros(12,1);
    tspan = [0 10];
    
    % Valores deseados
    z_des = 5; 
    phi_des = pi/4; 
    theta_des = pi/4; 
    psi_des = 0;
    
    try
        % Simular sistema
        [t, X] = ode45(@(t,X) dinamica_quadrotor(t, X, m, g, Ix, Iy, Iz, ...
                        ganancias, [z_des, phi_des, theta_des, psi_des]), tspan, X0);
        
        % Calcular errores
        error_z = abs(z_des - X(:,3));
        error_phi = abs(phi_des - X(:,4));
        error_theta = abs(theta_des - X(:,5));
        error_psi = abs(psi_des - X(:,6));
        
        % Penalización por sobreimpulso (más del 20%)
        sobreimpulso = max(0, max(X(:,3)) - 1.2*z_des);
        penalizacion_sobreimpulso = 10 * sobreimpulso;
        
        % Error en estado estacionario (último 10% de la simulación)
        idx_final = round(0.9*length(t)):length(t);
        error_estacionario = mean(error_z(idx_final)) + ...
                            mean(error_phi(idx_final)) + ...
                            mean(error_theta(idx_final)) + ...
                            mean(error_psi(idx_final));
        
        % Cálculo de ITAE (Integral del Tiempo por el Error Absoluto)
        itae_z = trapz(t, t.*error_z);
        itae_phi = trapz(t, t.*error_phi);
        itae_theta = trapz(t, t.*error_theta);
        itae_psi = trapz(t, t.*error_psi);
        
        % Fitness combinado
        fitness = itae_z + itae_phi + itae_theta + itae_psi + ...
                 penalizacion_sobreimpulso + 10*error_estacionario;
             
    catch
        % En caso de error en la simulación, asignar un fitness muy alto
        fitness = 1e4 + norm(ganancias);
    end
end

%% ================= DINÁMICA DEL QUADROTOR MEJORADA =================
function dXdt = dinamica_quadrotor(~, X, m, g, Ix, Iy, Iz, ganancias, deseado)
    persistent integral_z integral_phi integral_theta integral_psi
    if isempty(integral_z)
        integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;
    end
    
    % Extraer estados
    pos = X(1:6);   % [x, y, z, φ, θ, ψ]
    vel = X(7:12);   % Velocidades lineales y angulares
    
    % Valores deseados
    z_des = deseado(1); phi_des = deseado(2);
    theta_des = deseado(3); psi_des = deseado(4);
    
    % Cálculo de errores
    errores = [z_des - pos(3);    % Error altitud
              phi_des - pos(4);   % Error Roll
              theta_des - pos(5); % Error Pitch
              psi_des - pos(6)];  % Error Yaw
    
    % Actualizar integrales (con anti-windup básico)
    max_int = 10;
    integral_z = max(min(integral_z + errores(1), max_int), -max_int);   
    integral_phi = max(min(integral_phi + errores(2), max_int), -max_int); 
    integral_theta = max(min(integral_theta + errores(3), max_int), -max_int); 
    integral_psi = max(min(integral_psi + errores(4), max_int), -max_int);
    
    % Control PID completo (con acción integral)
    U1 = ganancias(1)*errores(1) + ganancias(2)*integral_z + ganancias(3)*(-vel(3));
    U2 = ganancias(4)*errores(2) + ganancias(5)*integral_phi + ganancias(6)*(-vel(4));
    U3 = ganancias(7)*errores(3) + ganancias(8)*integral_theta + ganancias(9)*(-vel(5));
    U4 = ganancias(10)*errores(4) + ganancias(11)*integral_psi + ganancias(12)*(-vel(6));
    
    % Limitar fuerza de empuje
    U1 = max(0, min(U1, 2*m*g));  % Fuerza positiva y limitada
    
    % Dinámica traslacional (modelo completo)
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