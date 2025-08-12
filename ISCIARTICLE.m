function pso_pid_full_experiment()

%% --- Configuración de pruebas ---
combinaciones_deseadas = [
    1.0,  0.0,   0.0,    0.0;
    1.5,  0.1,  -0.1,    0.0;
    2.0, -0.2,   0.2,    0.0;
    1.0,  0.0,   0.0,    pi/4;
    0.5, -0.1,  -0.1,   -pi/6
];

% Fuerza constante (bias) en N aplicada en eje Z (positivo = hacia arriba).
% Modifica este valor según tus experimentos.
bias_force = -0.5;  % por ejemplo -0.5 N (emula viento que empuja hacia abajo)

% Número de ejecuciones PSO por cada combinación para análisis estadístico
num_pruebas = 30;

% Carpeta de salida
folder_out = 'Resultados_PSO';
if ~exist(folder_out,'dir'), mkdir(folder_out); end

% Corre todas las combinaciones
for iComb = 1:size(combinaciones_deseadas,1)
    fprintf('\n============ Prueba %d ============\n', iComb);
    z_des = combinaciones_deseadas(iComb,1);
    phi_des = combinaciones_deseadas(iComb,2);
    theta_des = combinaciones_deseadas(iComb,3);
    psi_des = combinaciones_deseadas(iComb,4);

    % Nombres de archivos
    nombre_excel = fullfile(folder_out, sprintf('Resultados_PSO_PID_Test_%d.xlsx', iComb));
    nombre_figura_fitness = fullfile(folder_out, sprintf('Convergencia_Test_%d.png', iComb));
    nombre_figura_z = fullfile(folder_out, sprintf('RespuestaZ_Test_%d.png', iComb));
    nombre_figura_top5 = fullfile(folder_out, sprintf('Top5Trayectorias_Test_%d.png', iComb));

    % Ejecuta la rutina de optimización y guardado
    pso_pid_experiment(z_des, phi_des, theta_des, psi_des, bias_force, num_pruebas, ...
                      nombre_excel, nombre_figura_fitness, nombre_figura_z, nombre_figura_top5);
end

end

%% ------------------------------------------------------------------------
function pso_pid_experiment(z_des, phi_des, theta_des, psi_des, bias_force, num_pruebas, ...
                            nombre_archivo, nombre_figura, nombre_figura_z, nombre_figura_top5)
% Ejecuta num_pruebas ejecuciones del PSO donde la función objetivo PROMEDIA el
% desempeño en (sin perturbación) y (con perturbación constante = bias_force).

% Prealloc
resultados(num_pruebas) = struct();

% Para graficar convergencia promedio
all_convergences = [];

% Para guardar las mejores trayectorias de cada prueba
mejores_trayectorias = [];

for prueba = 1:num_pruebas
    fprintf('  Prueba %d/%d ...\n', prueba, num_pruebas);
    % Ejecuta PSO (optimiza sobre el fitness combinado)
    [global_best, convergence_curve, t_best_no, z_best_no, t_best_bias, z_best_bias, metrics_no, metrics_bias] = ...
        optimize_pid_pso(z_des, phi_des, theta_des, psi_des, bias_force);

    % Guarda resultados
    resultados(prueba).Prueba = prueba;
    resultados(prueba).Fitness = global_best.fitness;
    % Métricas sin perturbación (sufijo _no)
    resultados(prueba).RMSE_no = metrics_no.RMSE;
    resultados(prueba).Overshoot_no = metrics_no.overshoot;
    resultados(prueba).SettlingTime_no = metrics_no.t_settle;
    resultados(prueba).RiseTime_no = metrics_no.t_rise;
    resultados(prueba).SteadyError_no = metrics_no.steady_error;
    resultados(prueba).ITSE_no = metrics_no.ITSE;
    resultados(prueba).IAE_no = metrics_no.IAE;
    % Métricas con perturbación (sufijo _bias)
    resultados(prueba).RMSE_bias = metrics_bias.RMSE;
    resultados(prueba).Overshoot_bias = metrics_bias.overshoot;
    resultados(prueba).SettlingTime_bias = metrics_bias.t_settle;
    resultados(prueba).RiseTime_bias = metrics_bias.t_rise;
    resultados(prueba).SteadyError_bias = metrics_bias.steady_error;
    resultados(prueba).ITSE_bias = metrics_bias.ITSE;
    resultados(prueba).IAE_bias = metrics_bias.IAE;

    % Guardar ganancias
    gains = global_best.position;
    for k = 1:12
        resultados(prueba).(['g',num2str(k)]) = gains(k);
    end

    % Convergencia
    all_convergences = [all_convergences; convergence_curve(:)'];

    % Guardar trayectorias para graficar top5 (usaremos la trayectoria con bias)
    mejores_trayectorias(end+1).t = t_best_bias;
    mejores_trayectorias(end).z = z_best_bias;
    mejores_trayectorias(end).fitness = global_best.fitness; %#ok<AGROW>

    % Mensaje resumen
    fprintf('    Fitness: %.4f | RMSE_no: %.4f | RMSE_bias: %.4f\n', ...
            global_best.fitness, metrics_no.RMSE, metrics_bias.RMSE);
end

% Guardar Excel (tabla)
T = struct2table(resultados);
writetable(T, nombre_archivo);
fprintf('Archivo guardado: %s\n', nombre_archivo);

% Figura: convergencia promedio
avg_conv = mean(all_convergences,1);
figure('Visible','off');
plot(avg_conv,'LineWidth',2);
xlabel('Iteración'); ylabel('Fitness'); title('Convergencia promedio del PSO');
grid on; saveas(gcf, nombre_figura); close;

% Figura: mejor respuesta en z (elegimos la mejor prueba por fitness)
[~, idx_best] = min([resultados.Fitness]);
best_t = mejores_trayectorias(idx_best).t;
best_z = mejores_trayectorias(idx_best).z;
figure('Visible','off');
plot(best_t, best_z, 'LineWidth', 2); hold on;
yline(z_des,'--k','Valor deseado');
xlabel('Tiempo (s)'); ylabel('Altura z (m)');
title('Mejor respuesta en z (con perturbación)'); grid on;
saveas(gcf, nombre_figura_z); close;

% Figura: top5 trayectorias (con perturbación)
% Ordenamos por fitness
[~, idxs] = sort([mejores_trayectorias.fitness]);
topk = min(5, numel(idxs));
figure('Visible','off'); hold on;
for k = 1:topk
    idx = idxs(k);
    plot(mejores_trayectorias(idx).t, mejores_trayectorias(idx).z, 'LineWidth', 1.5);
end
yline(z_des,'--k','Valor deseado');
xlabel('Tiempo (s)'); ylabel('Altura z (m)');
title('Top 5 trayectorias (con perturbación)'); legend(arrayfun(@(n) sprintf('Prueba %d',n),1:topk,'UniformOutput',false));
grid on; saveas(gcf, nombre_figura_top5); close;

end

%% ------------------------------------------------------------------------
function [global_best, B, t_best_no, z_best_no, t_best_bias, z_best_bias, metrics_no, metrics_bias] = ...
         optimize_pid_pso(z_des, phi_des, theta_des, psi_des, bias_force)
% PSO que optimiza un fitness que es promedio (o suma ponderada) de
% desempeño en escenario NO PERT y escenario CON PERT (bias_force).

nVar = 12;
VarMin = [2.0 0.01 0.1  0.1 0.001 0.1  0.1 0.001 0.1  0.1 0.001 0.1];
VarMax = [15  2.0  5.0 10  0.1   2.0 10  0.1   2.0 10  0.1   2.0];
MaxIter = 100;
nPop = 50;
w = 0.7; d = 0.97; c1 = 1.7; c2 = 1.7;

% Inicialización
empty_particle = struct('position', [], 'velocity', [], 'fitness', [], 'best', []);
pop = repmat(empty_particle, nPop, 1);
global_best.fitness = inf;
B = zeros(MaxIter,1);

for i = 1:nPop
    pop(i).position = unifrnd(VarMin, VarMax);
    pop(i).velocity = zeros(1,nVar);
    % Evaluar fitness combinado (promedio de dos escenarios)
    pop(i).fitness = combined_fitness(pop(i).position, z_des, phi_des, theta_des, psi_des, bias_force);
    pop(i).best = pop(i);
    if pop(i).fitness < global_best.fitness
        global_best = pop(i).best;
    end
end

% PSO principal
for iter = 1:MaxIter
    for i = 1:nPop
        r1 = rand(1,nVar); r2 = rand(1,nVar);
        pop(i).velocity = w*pop(i).velocity + c1*r1.*(pop(i).best.position - pop(i).position) + ...
                          c2*r2.*(global_best.position - pop(i).position);
        pop(i).position = max(min(pop(i).position + pop(i).velocity, VarMax), VarMin);

        pop(i).fitness = combined_fitness(pop(i).position, z_des, phi_des, theta_des, psi_des, bias_force);

        if pop(i).fitness < pop(i).best.fitness
            pop(i).best.position = pop(i).position;
            pop(i).best.fitness = pop(i).fitness;
            if pop(i).best.fitness < global_best.fitness
                global_best = pop(i).best;
            end
        end
    end
    w = max(w*d, 0.4);
    B(iter) = global_best.fitness;
end

% Después de terminar PSO, obtengo las trayectorias y métricas para el mejor
% candidato, en ambos escenarios (sin perturbación y con bias)
% NOTA: evaluate_single_scenario devuelve [metrics, t, z]
[metrics_no, t_best_no, z_best_no]   = evaluate_single_scenario(global_best.position, z_des, phi_des, theta_des, psi_des, 0);
[metrics_bias, t_best_bias, z_best_bias] = evaluate_single_scenario(global_best.position, z_des, phi_des, theta_des, psi_des, bias_force);

end

%% ------------------------------------------------------------------------
function f = combined_fitness(ganancias, z_des, phi_des, theta_des, psi_des, bias_force)
% Calcula fitness combinado: evalúa métricas en escenario NO perturbación y en
% escenario CON perturbación (bias_force), normaliza y pondera.

% Pesos del fitness (puedes ajustarlos)
w_rmse = 0.25;
w_overshoot = 0.25;
w_settle = 0.20;
w_itse = 0.15;
w_iae = 0.15;

% Evalúa sin perturbación
[metrics_no, ~, ~] = evaluate_single_scenario(ganancias, z_des, phi_des, theta_des, psi_des, 0);
% Evalúa con perturbación
[metrics_bias, ~, ~] = evaluate_single_scenario(ganancias, z_des, phi_des, theta_des, psi_des, bias_force);

% Normalización heurística (divisor de referencia). Ajusta si hace falta.
% Evitamos divisiones por cero con eps.
rmse_norm_no = metrics_no.RMSE / (abs(z_des)+eps);
rmse_norm_bias = metrics_bias.RMSE / (abs(z_des)+eps);

overshoot_norm_no = metrics_no.overshoot / 100;
overshoot_norm_bias = metrics_bias.overshoot / 100;

settle_norm_no = metrics_no.t_settle / 10;
settle_norm_bias = metrics_bias.t_settle / 10;

itse_norm_no = metrics_no.ITSE / 50;
itse_norm_bias = metrics_bias.ITSE / 50;

iae_norm_no = metrics_no.IAE / 20;
iae_norm_bias = metrics_bias.IAE / 20;

% Fitness por escenario (suma ponderada)
fitness_no = w_rmse*min(rmse_norm_no,1) + w_overshoot*min(overshoot_norm_no,1) + ...
             w_settle*min(settle_norm_no,1) + w_itse*min(itse_norm_no,1) + w_iae*min(iae_norm_no,1);

fitness_bias = w_rmse*min(rmse_norm_bias,1) + w_overshoot*min(overshoot_norm_bias,1) + ...
               w_settle*min(settle_norm_bias,1) + w_itse*min(itse_norm_bias,1) + w_iae*min(iae_norm_bias,1);

% Fitness combinado: promedio (puedes cambiar por max, suma, etc.)
f = 0.5*(fitness_no + fitness_bias);

% Penalizaciones por valores no finitos
if ~isfinite(f)
    f = 1e3;
end
end

%% ------------------------------------------------------------------------
function [metrics, t, z] = evaluate_single_scenario(ganancias, z_des, phi_des, theta_des, psi_des, bias_force)
% Simula y calcula métricas para un escenario con una fuerza constante bias_force (N)
% bias_force = 0 --> sin perturbación.

% Parámetros físicos (puedes ajustar)
m = 1.0; g = 9.81; Ix = 0.1; Iy = 0.1; Iz = 0.2;
x0 = zeros(6,1); xdot0 = zeros(6,1); X0 = [x0; xdot0];
tspan = [0 10];

% Extrae ganancias
Kp_z = ganancias(1); Ki_z = ganancias(2); Kd_z = ganancias(3);
Kp_phi = ganancias(4); Ki_phi = ganancias(5); Kd_phi = ganancias(6);
Kp_theta = ganancias(7); Ki_theta = ganancias(8); Kd_theta = ganancias(9);
Kp_psi = ganancias(10); Ki_psi = ganancias(11); Kd_psi = ganancias(12);

% Inicializa estructura métricas
metrics = struct('t_settle', NaN, 'overshoot', NaN, 't_rise', NaN, 'steady_error', NaN, ...
                 'ITSE', NaN, 'IAE', NaN, 'RMSE', NaN);

try
    % Llamada a ODE (la dinámica considera bias_force)
    [t, X] = ode45(@(t,X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz, ...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi, ...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi, ...
        z_des, phi_des, theta_des, psi_des, bias_force), tspan, X0);

    z = X(:,3); error_z = z_des - z;

    % Settling time: heurística con tolerancia 2% del valor deseado
    tol = max(0.02*abs(z_des), 0.01);
    idx_settle = find(abs(error_z) > tol, 1, 'last');
    if isempty(idx_settle)
        metrics.t_settle = 0; % ya dentro de tolerancia
    else
        metrics.t_settle = t(min(idx_settle,length(t)));
    end

    % Overshoot en porcentaje (solo si z_des no es 0)
    if abs(z_des) > eps
        metrics.overshoot = max(0, (max(z) - z_des)/abs(z_des) * 100);
    else
        metrics.overshoot = 0;
    end

    % Rise time de 10% a 90%
    try
        t1 = t(find(z >= z_des*0.1, 1, 'first'));
        t2 = t(find(z >= z_des*0.9, 1, 'first'));
        metrics.t_rise = t2 - t1;
    catch
        metrics.t_rise = NaN;
    end

    % steady-state error promedio sobre el 10% final de la simulación
    metrics.steady_error = mean(abs(error_z(round(0.9*length(error_z)):end)));

    % ITSE e IAE
    metrics.ITSE = trapz(t, t .* (error_z).^2);
    metrics.IAE  = trapz(t, abs(error_z));
    metrics.RMSE = sqrt(mean(error_z.^2));

catch
    % En caso de error numérico, devolver valores grandes (penaliza)
    t = linspace(0,10,100)';
    z = zeros(size(t));
    metrics.t_settle = 100;
    metrics.overshoot = 1000;
    metrics.t_rise = 10;
    metrics.steady_error = 100;
    metrics.ITSE = 1e3;
    metrics.IAE = 1e3;
    metrics.RMSE = 1e3;
end

end

%% ------------------------------------------------------------------------
function dXdt = quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz, ...
    Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi, ...
    Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi, ...
    z_des, phi_des, theta_des, psi_des, bias_force)
% Dinámica simplificada del cuadricóptero con control PID.
% state X: [pos(1:6); vel(1:6)] donde pos = [x y z phi theta psi], vel = derivadas.

persistent iz ip it ipsi last_t
if isempty(iz)
    iz = 0; ip = 0; it = 0; ipsi = 0; last_t = t;
end

pos = X(1:6);
vel = X(7:12);

% Errores
err = [z_des - pos(3); phi_des - pos(4); theta_des - pos(5); psi_des - pos(6)];

% Integradores (anti-windup con saturación)
max_int = 10;
dt = max(t - last_t, 1e-6);
iz = max(min(iz + err(1)*dt, max_int), -max_int);
ip = max(min(ip + err(2)*dt, max_int), -max_int);
it = max(min(it + err(3)*dt, max_int), -max_int);
ipsi = max(min(ipsi + err(4)*dt, max_int), -max_int);
last_t = t;

% PID (derivada aproximada con -vel)
U1 = Kp_z*err(1) + Ki_z*iz + Kd_z*(-vel(3));
U2 = Kp_phi*err(2) + Ki_phi*ip + Kd_phi*(-vel(4));
U3 = Kp_theta*err(3) + Ki_theta*it + Kd_theta*(-vel(5));
U4 = Kp_psi*err(4) + Ki_psi*ipsi + Kd_psi*(-vel(6));

% Aceleraciones lineales (modelo simplificado)
acc_lin = [
    (cos(pos(4))*sin(pos(5))*cos(pos(6))+sin(pos(4))*sin(pos(6))) * U1 / m;
    (cos(pos(4))*sin(pos(5))*sin(pos(6))-sin(pos(4))*cos(pos(6))) * U1 / m;
    (cos(pos(4))*cos(pos(5)) * U1 / m) - g
];

% Agregamos la fuerza constante (bias_force) convertida a aceleración
% Si bias_force es fuerza (N), la aceleración adicional es bias_force / m.
if abs(bias_force) > 0
    acc_lin(3) = acc_lin(3) + bias_force / m;
end

% Aceleraciones angulares (modelo simplificado)
acc_ang = [
    (U2 + (Iy - Iz)*vel(5)*vel(6)) / Ix;
    (U3 + (Iz - Ix)*vel(4)*vel(6)) / Iy;
    (U4 + (Ix - Iy)*vel(4)*vel(5)) / Iz
];

dXdt = [vel; acc_lin; acc_ang];
end
