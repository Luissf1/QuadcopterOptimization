% Parámetros del quadrotor
m = 1.0;           % Masa (kg)
g = 9.81;          % Gravedad (m/s^2)
Ix = 0.1; Iy = 0.1; Iz = 0.2;  % Inercias (kg·m^2)

% Valores deseados
z_des = 5;         % Altura deseada (m)
phi_des = pi/4;    % Ángulo roll deseado (rad)
theta_des = pi/4;  % Ángulo pitch deseado (rad)
psi_des = 0;       % Ángulo yaw deseado (rad)

% Tiempo de simulación
tspan = [0 10];    % Tiempo inicial y final (s)

%% ================= OPTIMIZACIÓN PSO =================
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
    
    % Graficar convergencia
    figure;
    plot(convergencia, 'b-o', 'LineWidth', 2);
    xlabel('Iteración'); ylabel('Fitness');
    title('Convergencia del PSO'); grid on;
end

function simular_con_ganancias(ganancias, m, g, I, deseado, tspan)
    % Extraer parámetros
    Ix = I(1); Iy = I(2); Iz = I(3);
    z_des = deseado(1); phi_des = deseado(2);
    theta_des = deseado(3); psi_des = deseado(4);
    
    % Condiciones iniciales
    X0 = zeros(12,1);  % [posiciones; velocidades]
    
    % Simular
    [t, X] = ode45(@(t,X) dinamica_quadrotor(t, X, m, g, Ix, Iy, Iz, ...
                    ganancias, deseado), tspan, X0);
    
    % Graficar resultados
    figure;
    
    % Altura (z)
    subplot(2,2,1);
    plot(t, X(:,3)), hold on, plot(t, z_des*ones(size(t)), 'r--');
    title('Altura (z)'); xlabel('Tiempo (s)'); ylabel('m');
    legend('Real', 'Deseado');
    
    % Roll (φ)
    subplot(2,2,2);
    plot(t, X(:,4)), hold on, plot(t, phi_des*ones(size(t)), 'r--');
    title('Roll (φ)'); xlabel('Tiempo (s)'); ylabel('rad');
    
    % Pitch (θ)
    subplot(2,2,3);
    plot(t, X(:,5)), hold on, plot(t, theta_des*ones(size(t)), 'r--');
    title('Pitch (θ)'); xlabel('Tiempo (s)'); ylabel('rad');
    
    % Yaw (ψ)
    subplot(2,2,4);
    plot(t, X(:,6)), hold on, plot(t, psi_des*ones(size(t)), 'r--');
    title('Yaw (ψ)'); xlabel('Tiempo (s)'); ylabel('rad');
end

%% ================= ALGORITMO PSO =================
function [mejor_ganancia, convergencia] = optimizar_pid()
    % Configuración del PSO
    n_variables = 12;  % 4 controladores × 3 parámetros (Kp, Ki, Kd)
    iteraciones = 15;   % Número de iteraciones
    poblacion = 15;     % Tamaño de la población
    
    % Límites para los parámetros PID
    limites_min = [1.0  0.005  0.05  0.1  0.001  0.1  0.1  0.001  0.1  0.1  0.001  0.1];
    limites_max = [20   1     10    10   0.1    5    10   0.1    5    10   0.1    5];
    
    % Inicialización
    mejor_ganancia.posicion = [];
    mejor_ganancia.fitness = inf;
    convergencia = zeros(iteraciones, 1);
    
    % Crear población inicial
    particulas = struct('posicion', [], 'velocidad', [], 'fitness', [], 'mejor_local', []);
    for i = 1:poblacion
        particulas(i).posicion = limites_min + rand(1,n_variables).*(limites_max-limites_min);
        particulas(i).velocidad = zeros(1,n_variables);
        particulas(i).fitness = evaluar_pid(particulas(i).posicion);
        particulas(i).mejor_local = particulas(i);
        
        if particulas(i).fitness < mejor_ganancia.fitness
            mejor_ganancia = particulas(i).mejor_local;
        end
    end
    
    % Parámetros PSO
    inercia = 1.0;       % Peso de la velocidad anterior
    c1 = 2.5; c2 = 1.5;  % Factores cognitivo y social
    
    % Bucle principal
    for iter = 1:iteraciones
        for i = 1:poblacion
            % Actualizar velocidad
            r1 = rand(1,n_variables);
            r2 = rand(1,n_variables);
            
            particulas(i).velocidad = inercia * particulas(i).velocidad + ...
                c1 * r1 .* (particulas(i).mejor_local.posicion - particulas(i).posicion) + ...
                c2 * r2 .* (mejor_ganancia.posicion - particulas(i).posicion);
            
            % Actualizar posición
            particulas(i).posicion = particulas(i).posicion + particulas(i).velocidad;
            
            % Aplicar límites
            particulas(i).posicion = max(particulas(i).posicion, limites_min);
            particulas(i).posicion = min(particulas(i).posicion, limites_max);
            
            % Evaluar
            particulas(i).fitness = evaluar_pid(particulas(i).posicion);
            
            % Actualizar mejor local y global
            if particulas(i).fitness < particulas(i).mejor_local.fitness
                particulas(i).mejor_local = particulas(i);
                
                if particulas(i).fitness < mejor_ganancia.fitness
                    mejor_ganancia = particulas(i).mejor_local;
                end
            end
        end
        
        % Reducir inercia
        inercia = inercia * 0.98;
        
        % Guardar mejor fitness
        convergencia(iter) = mejor_ganancia.fitness;
        fprintf('Iteración %2d: Mejor fitness = %.2f\n', iter, convergencia(iter));
    end
    
    mejor_ganancia = mejor_ganancia.posicion;
end

%% ================= FUNCIÓN DE EVALUACIÓN =================
function fitness = evaluar_pid(ganancias)
    % Parámetros fijos (podrían pasarse como argumento)
    m = 1.0; g = 9.81; Ix = 0.1; Iy = 0.1; Iz = 0.2;
    z_des = 3; phi_des = pi/4; theta_des = pi/4; psi_des = 0;
    tspan = [0 10];
    X0 = zeros(12,1);
    
    try
        % Simular sistema
        [t, X] = ode45(@(t,X) dinamica_quadrotor(t, X, m, g, Ix, Iy, Iz, ...
                        ganancias, [z_des, phi_des, theta_des, psi_des]), tspan, X0);
        
        % Calcular errores
        error_z = abs(X(:,3) - z_des);
        error_phi = abs(X(:,4) - phi_des);
        
        % Calcular ITAE (Integral del tiempo por error absoluto)
        itae_z = trapz(t, t.*error_z);
        itae_phi = trapz(t, t.*error_phi);
        
        % Penalizar sobreimpulso
        sobreimpulso = max(0, max(X(:,3)) - 1.2*z_des); % >20% de sobreimpulso
        penalizacion = 100 * sobreimpulso^2;
        
        % Fitness combinado
        fitness = itae_z + itae_phi + penalizacion;
        
    catch
        % Si hay error (ganancias inestables)
        fitness = 1e6; % Valor muy alto
    end
end

%% ================= DINÁMICA DEL QUADROTOR =================
function dXdt = dinamica_quadrotor(~, X, m, g, Ix, Iy, Iz, ganancias, deseado)
    % Extraer estados
    pos = X(1:6);   % [x, y, z, φ, θ, ψ]
    vel = X(7:12);   % Velocidades lineales y angulares
    
    % Valores deseados
    z_des = deseado(1); phi_des = deseado(2);
    theta_des = deseado(3); psi_des = deseado(4);
    
    % Errores
    e_z = z_des - pos(3);
    e_phi = phi_des - pos(4);
    e_theta = theta_des - pos(5);
    e_psi = psi_des - pos(6);
    
    % Control PID 
    U1 = ganancias(1)*e_z + ganancias(3)*(-vel(3));  % Altura
    U2 = ganancias(4)*e_phi + ganancias(6)*(-vel(4)); % Roll
    U3 = ganancias(7)*e_theta + ganancias(9)*(-vel(5)); % Pitch
    U4 = ganancias(10)*e_psi + ganancias(12)*(-vel(6)); % Yaw
    
    % Limitar fuerzas
    U1 = max(0, min(U1, 2*m*g));  % Fuerza positiva y limitada
    
    % Dinámica traslacional 
    acc_lin = [0; 0; U1/m - g];
    
    % Dinámica rotacional
    acc_ang = [
        (U2 + (Iy-Iz)*vel(5)*vel(6))/Ix;
        (U3 + (Iz-Ix)*vel(4)*vel(6))/Iy;
        (U4 + (Ix-Iy)*vel(4)*vel(5))/Iz];
    
    % Derivadas del estado
    dXdt = [vel; acc_lin; acc_ang];
end