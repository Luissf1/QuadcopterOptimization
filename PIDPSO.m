function pso_pid_multiple_tests()
    % Combinaciones deseadas: [z, phi, theta, psi]
    combinaciones_deseadas = [...
        1.0,  0.0,   0.0,    0.0;
        1.5,  0.1,  -0.1,    0.0;
        2.0, -0.2,   0.2,    0.0;
        1.0,  0.0,   0.0,    pi/4;
        0.5, -0.1,  -0.1,   -pi/6];

    for i = 1:size(combinaciones_deseadas, 1)
        fprintf('\n============ Prueba %d ============\n', i);
        z_des = combinaciones_deseadas(i,1);
        phi_des = combinaciones_deseadas(i,2);
        theta_des = combinaciones_deseadas(i,3);
        psi_des = combinaciones_deseadas(i,4);

        nombre_excel = sprintf('Resultados_PSO_PID_Test_%d.xlsx', i);
        nombre_figura_fitness = sprintf('Convergencia_Test_%d.png', i);
        nombre_figura_z = sprintf('RespuestaZ_Test_%d.png', i);

        pso_pid_optimization_with_metrics(z_des, phi_des, theta_des, psi_des, ...
                                          nombre_excel, nombre_figura_fitness, nombre_figura_z);
    end
end

function pso_pid_optimization_with_metrics(z_des, phi_des, theta_des, psi_des, nombre_archivo, nombre_figura, nombre_figura_z)
    num_pruebas = 30;
    resultados(num_pruebas,1) = struct();
    best_fitness_over_time = [];
    best_global_overall.fitness = inf;
    mejores_resultados = struct('t', {}, 'z', {}, 'label', {});

    for prueba = 1:num_pruebas
        [global_best, metrics, convergence_fitness, t_best, z_best] = ...
            optimize_pid_with_pso_and_metrics(z_des, phi_des, theta_des, psi_des);

        best_fitness_over_time = [best_fitness_over_time; convergence_fitness(:)'];

        resultados(prueba).Prueba = prueba;
        resultados(prueba).Fitness = global_best.fitness;
        resultados(prueba).SettlingTime = metrics.t_settle;
        resultados(prueba).Overshoot = metrics.overshoot;
        resultados(prueba).RiseTime = metrics.t_rise;
        resultados(prueba).SteadyError = metrics.steady_error;
        resultados(prueba).ITSE = metrics.ITSE;
        resultados(prueba).IAE = metrics.IAE;
        resultados(prueba).RMSE = metrics.RMSE;
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

        if global_best.fitness < best_global_overall.fitness
            best_global_overall = global_best;
            t_global_best = t_best;
            z_global_best = z_best;
        end

        mejores_resultados(end+1).t = t_best;
        mejores_resultados(end).z = z_best;
        mejores_resultados(end).label = sprintf('Prueba %d', prueba);

        fprintf('%d\t%.4f\t%.4f\t%.2f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n', ...
                prueba, global_best.fitness, metrics.t_settle, metrics.overshoot, ...
                metrics.t_rise, metrics.steady_error, metrics.ITSE, metrics.IAE, metrics.RMSE);
    end

    % Guardar resultados a Excel
    tabla = struct2table(resultados);
    writetable(tabla, nombre_archivo);
    fprintf('\nArchivo guardado: %s\n', nombre_archivo);

    % Figura de convergencia promedio
    avg_convergence = mean(best_fitness_over_time, 1);
    figure;
    plot(avg_convergence, 'b-', 'LineWidth', 2);
    xlabel('Iteración');
    ylabel('Fitness');
    title('Convergencia promedio del PSO');
    grid on;
    saveas(gcf, nombre_figura);
    close;

    % Figura respuesta Z vs t
    figure;
    plot(t_global_best, z_global_best, 'r', 'LineWidth', 2);
    yline(z_des, '--k', 'Valor deseado');
    xlabel('Tiempo (s)');
    ylabel('Altura z (m)');
    title('Respuesta en Altura z para la mejor solución');
    grid on;
    saveas(gcf, nombre_figura_z);
    close;

    % Figura de las 5 mejores trayectorias
    [~, idx_sort] = sort([resultados.Fitness]);
    mejores_resultados = mejores_resultados(idx_sort);
    mejores_resultados = mejores_resultados(1:min(5,end));

    figure;
    hold on;
    for i = 1:length(mejores_resultados)
        plot(mejores_resultados(i).t, mejores_resultados(i).z, 'LineWidth', 1.5);
    end
    yline(z_des, '--k', 'Valor deseado');
    xlabel('Tiempo (s)');
    ylabel('Altura z (m)');
    title('Comparación de las 5 mejores trayectorias');
    legend({mejores_resultados.label}, 'Location', 'best');
    grid on;
    saveas(gcf, strrep(nombre_figura_z, '.png', '_Top5.png'));
    close;
end

function [global_best, metrics, B, t_best, z_best] = optimize_pid_with_pso_and_metrics(z_des, phi_des, theta_des, psi_des)
    nVar = 12;
    VarMin = [2.0 0.01 0.1  0.1 0.001 0.1  0.1 0.001 0.1  0.1 0.001 0.1];
    VarMax = [15  2.0  5.0 10  0.1   2.0 10  0.1   2.0 10  0.1   2.0];
    MaxIter = 100;
    nPop = 50;
    w = .7; d = 0.97; c1 = 1.7; c2 = 1.7;

    empty_particle = struct('position', [], 'velocity', [], 'fitness', [], 'best', []);
    pop = repmat(empty_particle, nPop, 1);
    global_best.fitness = inf;
    B = zeros(MaxIter, 1);
    t_best = [];
    z_best = [];

    for i = 1:nPop
        pop(i).position = unifrnd(VarMin, VarMax);
        pop(i).velocity = zeros(1, nVar);
        [pop(i).fitness, ~, ~, ~] = evaluate_pid(pop(i).position, z_des, phi_des, theta_des, psi_des);
        pop(i).best = pop(i);
        if pop(i).fitness < global_best.fitness
            global_best = pop(i).best;
        end
    end

    for iter = 1:MaxIter
        for i = 1:nPop
            r1 = rand(1, nVar);
            r2 = rand(1, nVar);
            pop(i).velocity = w * pop(i).velocity + ...
                c1 * r1 .* (pop(i).best.position - pop(i).position) + ...
                c2 * r2 .* (global_best.position - pop(i).position);
            pop(i).position = max(min(pop(i).position + pop(i).velocity, VarMax), VarMin);
            [pop(i).fitness, temp_metrics, t, z] = evaluate_pid(pop(i).position, z_des, phi_des, theta_des, psi_des);
            if pop(i).fitness < pop(i).best.fitness
                pop(i).best.position = pop(i).position;
                pop(i).best.fitness = pop(i).fitness;
                if pop(i).best.fitness < global_best.fitness
                    global_best = pop(i).best;
                    metrics = temp_metrics;
                    t_best = t;
                    z_best = z;
                end
            end
        end
        w = max(w * d, 0.4);
        B(iter) = global_best.fitness;
    end
end

function [fitness, metrics, t, z] = evaluate_pid(ganancias, z_des, phi_des, theta_des, psi_des)
    m = 1.0; g = 9.81; Ix = 0.1; Iy = 0.1; Iz = 0.2;
    x0 = zeros(6,1); xdot0 = zeros(6,1); X0 = [x0; xdot0];
    tspan = [0 10];

    Kp_z    = ganancias(1);  Ki_z    = ganancias(2);  Kd_z    = ganancias(3);
    Kp_phi  = ganancias(4);  Ki_phi  = ganancias(5);  Kd_phi  = ganancias(6);
    Kp_theta= ganancias(7);  Ki_theta= ganancias(8);  Kd_theta= ganancias(9);
    Kp_psi  = ganancias(10); Ki_psi  = ganancias(11); Kd_psi  = ganancias(12);

    metrics = struct('t_settle', NaN, 'overshoot', NaN, 't_rise', NaN, ...
                     'steady_error', NaN, 'ITSE', NaN, 'IAE', NaN, 'RMSE', NaN);
    try
        [t, X] = ode45(@(t,X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz, ...
            Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi, ...
            Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi, ...
            z_des, phi_des, theta_des, psi_des), tspan, X0);

        z = X(:,3);
        error_z = z_des - z;

        tol = 0.02 * z_des;
        idx_settle = find(abs(error_z) > tol, 1, 'last');
        metrics.t_settle = isempty(idx_settle) * 0 + ~isempty(idx_settle) * t(idx_settle);
        metrics.overshoot = max(0, (max(z) - z_des)/z_des * 100);
        rise_start = z_des * 0.1;
        rise_end = z_des * 0.9;
        try
            t_rise_start = t(find(z >= rise_start, 1));
            t_rise_end = t(find(z >= rise_end, 1));
            metrics.t_rise = t_rise_end - t_rise_start;
        catch
            metrics.t_rise = NaN;
        end
        metrics.steady_error = mean(abs(error_z(round(0.9*end):end)));
        metrics.ITSE = trapz(t, t .* (error_z).^2);
        metrics.IAE = trapz(t, abs(error_z));
        metrics.RMSE = sqrt(mean(error_z.^2));

        fitness = 0.3*min(metrics.t_settle/10,1) + ...
                  0.3*min(metrics.overshoot/100,1) + ...
                  0.2*min(metrics.ITSE/50,1) + ...
                  0.2*min(metrics.IAE/20,1);
    catch
        fitness = 1000;
        metrics = struct('t_settle', 100, 'overshoot', 1000, 't_rise', 10, ...
                         'steady_error', 1, 'ITSE', 50, 'IAE', 20, 'RMSE', 50);
        t = linspace(0,10,100); z = zeros(size(t));
    end
end

function dXdt = quadrotor_dynamics(~, X, m, g, Ix, Iy, Iz,...
    Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
    Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
    z_des, phi_des, theta_des, psi_des)

    persistent iz ip it ipsi
    if isempty(iz), iz = 0; ip = 0; it = 0; ipsi = 0; end
    pos = X(1:6); vel = X(7:12);
    err = [z_des - pos(3); phi_des - pos(4); theta_des - pos(5); psi_des - pos(6)];
    max_int = 10;
    iz = max(min(iz + err(1), max_int), -max_int);
    ip = max(min(ip + err(2), max_int), -max_int);
    it = max(min(it + err(3), max_int), -max_int);
    ipsi = max(min(ipsi + err(4), max_int), -max_int);

    U1 = Kp_z*err(1) + Ki_z*iz + Kd_z*(-vel(3));
    U2 = Kp_phi*err(2) + Ki_phi*ip + Kd_phi*(-vel(4));
    U3 = Kp_theta*err(3) + Ki_theta*it + Kd_theta*(-vel(5));
    U4 = Kp_psi*err(4) + Ki_psi*ipsi + Kd_psi*(-vel(6));

    acc_lin = [(cos(pos(4))*sin(pos(5))*cos(pos(6))+sin(pos(4))*sin(pos(6)))*U1/m;
               (cos(pos(4))*sin(pos(5))*sin(pos(6))-sin(pos(4))*cos(pos(6)))*U1/m;
               (cos(pos(4))*cos(pos(5))*U1/m) - g];
    acc_ang = [(U2 + (Iy - Iz)*vel(5)*vel(6))/Ix;
               (U3 + (Iz - Ix)*vel(4)*vel(6))/Iy;
               (U4 + (Ix - Iy)*vel(4)*vel(5))/Iz];
    dXdt = [vel; acc_lin; acc_ang];
end
