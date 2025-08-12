% Run the optimization
pso_pid_optimization_with_metrics();

function pso_pid_optimization_with_metrics()
    % Número de pruebas a realizar
    num_pruebas = 30;

    % Preparar estructura para almacenar resultados
    resultados = struct('Prueba', cell(num_pruebas,1), ...
                        'Fitness', cell(num_pruebas,1), ...
                        'SettlingTime', cell(num_pruebas,1), ...
                        'Overshoot', cell(num_pruebas,1), ...
                        'RiseTime', cell(num_pruebas,1), ...
                        'SteadyError', cell(num_pruebas,1), ...
                        'ITSE', cell(num_pruebas,1), ...
                        'IAE', cell(num_pruebas,1), ...
                        'Kp_z', cell(num_pruebas,1), ...
                        'Ki_z', cell(num_pruebas,1), ...
                        'Kd_z', cell(num_pruebas,1), ...
                        'Kp_phi', cell(num_pruebas,1), ...
                        'Ki_phi', cell(num_pruebas,1), ...
                        'Kd_phi', cell(num_pruebas,1), ...
                        'Kp_theta', cell(num_pruebas,1), ...
                        'Ki_theta', cell(num_pruebas,1), ...
                        'Kd_theta', cell(num_pruebas,1), ...
                        'Kp_psi', cell(num_pruebas,1), ...
                        'Ki_psi', cell(num_pruebas,1), ...
                        'Kd_psi', cell(num_pruebas,1));

    % Encabezado para la consola
    fprintf('=== PSO-PID Optimization with Performance Metrics ===\n');
    fprintf('Running %d iterations...\n', num_pruebas);
    fprintf('Prueba\tFitness\t\tSettling(s)\tOvershoot(%%)\tRise(s)\tSteadyError\tITSE\t\tIAE\n');
    fprintf('==================================================================================\n');

    % Ejecutar las pruebas
    for prueba = 1:num_pruebas
        fprintf('Running test %d/%d... ', prueba, num_pruebas);
        
        % Optimización
        [global_best, metrics] = optimize_pid_with_pso_and_metrics();
        
        % Almacenar resultados
        resultados(prueba).Prueba = prueba;
        resultados(prueba).Fitness = global_best.fitness;
        resultados(prueba).SettlingTime = metrics.t_settle;
        resultados(prueba).Overshoot = metrics.overshoot;
        resultados(prueba).RiseTime = metrics.t_rise;
        resultados(prueba).SteadyError = metrics.steady_error;
        resultados(prueba).ITSE = metrics.ITSE;
        resultados(prueba).IAE = metrics.IAE;
        resultados(prueba).Kp_z = global_best.position(1);
        resultados(prueba).Ki_z = global_best.position(2);
        resultados(prueba).Kd_z = global_best.position(3);
        resultados(prueba).Kp_phi = global_best.position(4);
        resultados(prueba).Ki_phi = global_best.position(5);
        resultados(prueba).Kd_phi = global_best.position(6);
        resultados(prueba).Kp_theta = global_best.position(7);
        resultados(prueba).Ki_theta = global_best.position(8);
        resultados(prueba).Kd_theta = global_best.position(9);
        resultados(prueba).Kp_psi = global_best.position(10);
        resultados(prueba).Ki_psi = global_best.position(11);
        resultados(prueba).Kd_psi = global_best.position(12);
        
        % Mostrar en consola
        fprintf('%d\t%.4f\t%.4f\t%.2f\t\t%.4f\t%.4f\t%.4f\t%.4f\n', ...
                prueba, global_best.fitness, metrics.t_settle, metrics.overshoot, ...
                metrics.t_rise, metrics.steady_error, metrics.ITSE, metrics.IAE);
    end

    % Convertir estructura a tabla
    tabla_resultados = struct2table(resultados);

    % Guardar en archivo Excel
    nombre_archivo = 'PSO_PID_Optimization_Results.xlsx';
    writetable(tabla_resultados, nombre_archivo);
    fprintf('\nResults saved to: %s\n', nombre_archivo);

    % Mostrar estadísticas
    show_statistics(resultados);
    
    % Comparación con Ziegler-Nichols
    compare_with_ziegler_nichols(resultados);
end

function [global_best, metrics] = optimize_pid_with_pso_and_metrics()
    % Parámetros del PSO
    nVar = 12;
    VarMin = [2.0  0.01  0.1 ... 
              0.1  0.001  0.1 ...  
              0.1  0.001  0.1 ...  
              0.1  0.001  0.1];    

    VarMax = [15   2.0   5 ...    
              10   0.1   2 ...    
              10   0.1   2 ...    
              10   0.1   2]; 
  
    MaxIter = 100;
    nPop = 50;
    w = .7;
    d = 0.97;
    c1 = 1.7;
    c2 = 1.7;

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
        [pop(i).fitness, ~] = evaluate_pid(pop(i).position);
        pop(i).best = pop(i);
        
        if pop(i).fitness < global_best.fitness
            global_best = pop(i).best;
        end
    end

    % Bucle principal
    for iter = 1:MaxIter
        for i = 1:nPop
            % Actualizar velocidad y posición
            r1 = rand(1, nVar);
            r2 = rand(1, nVar);
            
            pop(i).velocity = w * pop(i).velocity + ...
                             c1 * r1 .* (pop(i).best.position - pop(i).position) + ...
                             c2 * r2 .* (global_best.position - pop(i).position);
            
            pop(i).position = pop(i).position + pop(i).velocity;
            pop(i).position = max(pop(i).position, VarMin);
            pop(i).position = min(pop(i).position, VarMax);
            
            % Evaluar
            [pop(i).fitness, temp_metrics] = evaluate_pid(pop(i).position);
            
            % Actualizar mejores
            if pop(i).fitness < pop(i).best.fitness
                pop(i).best.position = pop(i).position;
                pop(i).best.fitness = pop(i).fitness;
                
                if pop(i).best.fitness < global_best.fitness
                    global_best = pop(i).best;
                    metrics = temp_metrics; % Store metrics from best solution
                end
            end
        end
        
        % Actualizar inercia
        w = max(w * d, 0.4);
        B(iter) = global_best.fitness;
    end
end

function [fitness, metrics] = evaluate_pid(ganancias)
    % Parámetros fijos del cuadricóptero
    m = 1.0; g = 9.81; 
    Ix = 0.1; Iy = 0.1; Iz = 0.2;
    
    % Condiciones iniciales
    x0 = zeros(6,1); xdot0 = zeros(6,1);
    X0 = [x0; xdot0];
    tspan = [0 10];
    
    % Valores deseados (se modifican dependiendo de maneobra)
    z_des = 1.5; 
    phi_des = 0.1; 
    theta_des = -0.1; 
    psi_des = 0;
    
    % Desempaquetar ganancias
    Kp_z = ganancias(1); Ki_z = ganancias(2); Kd_z = ganancias(3);
    Kp_phi = ganancias(4); Ki_phi = ganancias(5); Kd_phi = ganancias(6);
    Kp_theta = ganancias(7); Ki_theta = ganancias(8); Kd_theta = ganancias(9);
    Kp_psi = ganancias(10); Ki_psi = ganancias(11); Kd_psi = ganancias(12);
    
    % Inicializar métricas
    metrics = struct('t_settle', NaN, 'overshoot', NaN, 't_rise', NaN, ...
                    'steady_error', NaN, 'ITSE', NaN, 'IAE', NaN);
    
    % Simulación del sistema
    try
        [t, X] = ode45(@(t,X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
                          Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
                          Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
                          z_des, phi_des, theta_des, psi_des), tspan, X0);
        
        % Cálculo de métricas para altitud (z)
        z = X(:,3);
        error_z = z_des - z;
        
        % 1. Settling time (2% criterion)
        tolerance = 0.02 * z_des;
        idx_settle = find(abs(error_z) > tolerance, 1, 'last');
        if isempty(idx_settle)
            metrics.t_settle = 0;
        else
            metrics.t_settle = t(idx_settle);
        end
        
        % 2. Overshoot (%)
        metrics.overshoot = max(0, (max(z) - z_des)/z_des * 100);
        
        % 3. Rise time (10% to 90%)
        rise_start = z_des * 0.1;
        rise_end = z_des * 0.9;
        try
            t_rise_start = t(find(z >= rise_start, 1));
            t_rise_end = t(find(z >= rise_end, 1));
            metrics.t_rise = t_rise_end - t_rise_start;
        catch
            metrics.t_rise = NaN;
        end
        
        % 4. Steady-state error (last 10% of simulation)
        metrics.steady_error = mean(abs(error_z(round(0.9*end):end)));
        
        % 5. ITSE (Integral Time Squared Error)
        metrics.ITSE = trapz(t, t.*error_z.^2);
        
        % 6. IAE (Integral Absolute Error)
        metrics.IAE = trapz(t, abs(error_z));
        
        % Fitness function weights
        w_settle = 0.3;
        w_overshoot = 0.3;
        w_ITSE = 0.2;
        w_IAE = 0.2;
        
        % Normalized fitness components
        norm_t_settle = min(metrics.t_settle/10, 1);      % Max 10s
        norm_overshoot = min(metrics.overshoot/100, 1);    % Max 100%
        norm_ITSE = min(metrics.ITSE/50, 1);               % Empirical max
        norm_IAE = min(metrics.IAE/20, 1);                 % Empirical max
        
        % Combined fitness (lower is better)
        fitness = w_settle*norm_t_settle + ...
                 w_overshoot*norm_overshoot + ...
                 w_ITSE*norm_ITSE + ...
                 w_IAE*norm_IAE;
             
    catch
        % Penalización por fallo en simulación
        % fitness = 10; % Maximum penalty
        % metrics.t_settle = 10;
        % metrics.overshoot = 100;
        fitness = 1000; % Penalización muy alta
        metrics.t_settle = 100;
        metrics.overshoot = 1000;
        metrics.t_rise = 10;
        metrics.steady_error = 1;
        metrics.ITSE = 50;
        metrics.IAE = 20;
    end
end

function show_statistics(resultados)
    fprintf('\n=== Optimization Statistics ===\n');
    fprintf('Metric\t\t\tBest\t\tWorst\t\tMean\t\tStd Dev\n');
    fprintf('Fitness\t\t\t%.4f\t\t%.4f\t\t%.4f\t\t%.4f\n', ...
            min([resultados.Fitness]), max([resultados.Fitness]), ...
            mean([resultados.Fitness]), std([resultados.Fitness]));
    
    fprintf('Settling Time (s)\t%.4f\t\t%.4f\t\t%.4f\t\t%.4f\n', ...
            min([resultados.SettlingTime]), max([resultados.SettlingTime]), ...
            mean([resultados.SettlingTime]), std([resultados.SettlingTime]));
    
    fprintf('Overshoot (%%)\t\t%.2f\t\t%.2f\t\t%.2f\t\t%.2f\n', ...
            min([resultados.Overshoot]), max([resultados.Overshoot]), ...
            mean([resultados.Overshoot]), std([resultados.Overshoot]));
    
    % ... similar for other metrics ...
end

function compare_with_ziegler_nichols(resultados)
    % Valores de referencia Ziegler-Nichols (ejemplo)
    zn_metrics.t_settle = 4.25;
    zn_metrics.overshoot = 32.4;
    zn_metrics.t_rise = 1.85;
    zn_metrics.steady_error = 0.045;
    zn_metrics.ITSE = 1.24;
    zn_metrics.IAE = 0.87;
    
    % Calcular promedios PSO
    pso_avg.t_settle = mean([resultados.SettlingTime]);
    pso_avg.overshoot = mean([resultados.Overshoot]);
    pso_avg.t_rise = mean([resultados.RiseTime]);
    pso_avg.steady_error = mean([resultados.SteadyError]);
    pso_avg.ITSE = mean([resultados.ITSE]);
    pso_avg.IAE = mean([resultados.IAE]);
    
    % Calcular porcentajes de mejora
    improvement.t_settle = (zn_metrics.t_settle - pso_avg.t_settle)/zn_metrics.t_settle * 100;
    improvement.overshoot = (zn_metrics.overshoot - pso_avg.overshoot)/zn_metrics.overshoot * 100;
    improvement.t_rise = (zn_metrics.t_rise - pso_avg.t_rise)/zn_metrics.t_rise * 100;
    improvement.steady_error = (zn_metrics.steady_error - pso_avg.steady_error)/zn_metrics.steady_error * 100;
    improvement.ITSE = (zn_metrics.ITSE - pso_avg.ITSE)/zn_metrics.ITSE * 100;
    improvement.IAE = (zn_metrics.IAE - pso_avg.IAE)/zn_metrics.IAE * 100;
    
    % Mostrar tabla comparativa
    fprintf('\n=== PSO vs Ziegler-Nichols Comparison ===\n');
    fprintf('%-20s %-12s %-12s %-12s\n', 'Metric', 'ZN', 'PSO', 'Improvement');
    fprintf('%-20s %-12.4f %-12.4f %-12.1f%%\n', 'Settling Time (s)', ...
            zn_metrics.t_settle, pso_avg.t_settle, improvement.t_settle);
    fprintf('%-20s %-12.4f %-12.4f %-12.1f%%\n', 'Overshoot (%)', ...
            zn_metrics.overshoot, pso_avg.overshoot, improvement.overshoot);
    fprintf('%-20s %-12.4f %-12.4f %-12.1f%%\n', 'Rise Time (s)', ...
            zn_metrics.t_rise, pso_avg.t_rise, improvement.t_rise);
    fprintf('%-20s %-12.4f %-12.4f %-12.1f%%\n', 'Steady-State Error', ...
            zn_metrics.steady_error, pso_avg.steady_error, improvement.steady_error);
    fprintf('%-20s %-12.4f %-12.4f %-12.1f%%\n', 'ITSE', ...
            zn_metrics.ITSE, pso_avg.ITSE, improvement.ITSE);
    fprintf('%-20s %-12.4f %-12.4f %-12.1f%%\n', 'IAE', ...
            zn_metrics.IAE, pso_avg.IAE, improvement.IAE);
end

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
    max_int = 10;
    integral_z = max(min(integral_z + errores(1)), -max_int);
    integral_phi = max(min(integral_phi + errores(2)), -max_int);
    integral_theta = max(min(integral_theta + errores(3)), -max_int);
    integral_psi = max(min(integral_psi + errores(4)), -max_int);
    
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

