%% Complete Quadrotor PID Optimization with PSO
close all; clear all; clc;

%% Experiment Configuration
num_experiments = 30;
overshoot_data = zeros(num_experiments, 4);  % [z, phi, theta, psi]
ISE_data = zeros(num_experiments, 4);        % [z, phi, theta, psi]
settling_time_data = zeros(num_experiments, 4); % [z, phi, theta, psi]
best_gains = zeros(num_experiments, 12);     % Store all best gains

% Desired values
z_des = 1;          % Desired altitude [m]
phi_des = 0;        % Desired roll [rad]
theta_des = 0;      % Desired pitch [rad]
psi_des = pi/4;     % Desired yaw [rad]

%% Main Experiment Loop
for exp_num = 1:num_experiments
    fprintf('\n=== Experiment %d/%d ===\n', exp_num, num_experiments);
    
    % Run PSO optimization
    [global_best, convergence] = optimize_pid_with_pso();
    
    % Store best gains
    best_gains(exp_num,:) = global_best.position;
    
    % Extract optimal gains
    optimal_gains = global_best.position;
    Kp_z = optimal_gains(1); Ki_z = optimal_gains(2); Kd_z = optimal_gains(3);
    Kp_phi = optimal_gains(4); Ki_phi = optimal_gains(5); Kd_phi = optimal_gains(6);
    Kp_theta = optimal_gains(7); Ki_theta = optimal_gains(8); Kd_theta = optimal_gains(9);
    Kp_psi = optimal_gains(10); Ki_psi = optimal_gains(11); Kd_psi = optimal_gains(12);
    
    % System parameters
    m = 1.0; g = 9.81; Ix = 0.1; Iy = 0.1; Iz = 0.2;
    x0 = zeros(6,1); xdot0 = zeros(6,1); X0 = [x0; xdot0];
    tspan = [0 10];
    
    % Simulate system
    [t, X] = ode45(@(t,X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
                  Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
                  Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
                  z_des, phi_des, theta_des, psi_des), tspan, X0);
    
    %% Performance Metrics Calculation
    % 1. Overshoot calculation
    overshoot_z = max(0, (max(X(:,3))) - z_des)/z_des * 100;
    overshoot_phi = max(0, (max(abs(X(:,4)))) - abs(phi_des))/(abs(phi_des)+eps) * 100;
    overshoot_theta = max(0, (max(abs(X(:,5)))) - abs(theta_des))/(abs(theta_des)+eps) * 100;
    overshoot_psi = max(0, (max(abs(X(:,6)))) - abs(psi_des))/abs(psi_des) * 100;
    
    overshoot_data(exp_num,:) = [overshoot_z, overshoot_phi, overshoot_theta, overshoot_psi];
    
    % 2. ISE calculation
    error_z = (z_des - X(:,3)).^2;
    error_phi = (phi_des - X(:,4)).^2;
    error_theta = (theta_des - X(:,5)).^2;
    error_psi = (psi_des - X(:,6)).^2;
    
    ISE_data(exp_num,:) = [trapz(t, error_z), trapz(t, error_phi), ...
                         trapz(t, error_theta), trapz(t, error_psi)];
    
    % 3. Settling time calculation (2% criterion)
    [settling_z] = calculate_settling_time(t, X(:,3), z_des, 0.02);
    [settling_phi] = calculate_settling_time(t, X(:,4), phi_des, 0.02);
    [settling_theta] = calculate_settling_time(t, X(:,5), theta_des, 0.02);
    [settling_psi] = calculate_settling_time(t, X(:,6), psi_des, 0.02);
    
    % Ensure scalar values
    settling_time_data(exp_num,:) = [settling_z(1), settling_phi(1), settling_theta(1), settling_psi(1)];
    
    %% Display current experiment results
    fprintf('--- Performance Metrics ---\n');
    fprintf('Overshoot: z=%.2f%%, φ=%.2f%%, θ=%.2f%%, ψ=%.2f%%\n', ...
            overshoot_z, overshoot_phi, overshoot_theta, overshoot_psi);
    fprintf('ISE: z=%.4f, φ=%.4f, θ=%.4f, ψ=%.4f\n', ISE_data(exp_num,:));
    fprintf('Settling Time: z=%.2fs, φ=%.2fs, θ=%.2fs, ψ=%.2fs\n\n', ...
            settling_time_data(exp_num,:));
end

%% Final Results Analysis
fprintf('\n=== Final Results (%d experiments) ===\n', num_experiments);

% Overshoot statistics
fprintf('\nOvershoot Statistics [%%]:\n');
fprintf('        Mean    Std Dev\n');
fprintf('z:    %7.2f  %7.2f\n', mean(overshoot_data(:,1)), std(overshoot_data(:,1)));
fprintf('φ:    %7.2f  %7.2f\n', mean(overshoot_data(:,2)), std(overshoot_data(:,2)));
fprintf('θ:    %7.2f  %7.2f\n', mean(overshoot_data(:,3)), std(overshoot_data(:,3)));
fprintf('ψ:    %7.2f  %7.2f\n', mean(overshoot_data(:,4)), std(overshoot_data(:,4)));

% ISE statistics
fprintf('\nISE Statistics:\n');
fprintf('        Mean    Std Dev\n');
fprintf('z:    %7.4f  %7.4f\n', mean(ISE_data(:,1)), std(ISE_data(:,1)));
fprintf('φ:    %7.4f  %7.4f\n', mean(ISE_data(:,2)), std(ISE_data(:,2)));
fprintf('θ:    %7.4f  %7.4f\n', mean(ISE_data(:,3)), std(ISE_data(:,3)));
fprintf('ψ:    %7.4f  %7.4f\n', mean(ISE_data(:,4)), std(ISE_data(:,4)));

% Settling time statistics
fprintf('\nSettling Time Statistics [s]:\n');
fprintf('        Mean    Std Dev\n');
fprintf('z:    %7.2f  %7.2f\n', mean(settling_time_data(:,1)), std(settling_time_data(:,1)));
fprintf('φ:    %7.2f  %7.2f\n', mean(settling_time_data(:,2)), std(settling_time_data(:,2)));
fprintf('θ:    %7.2f  %7.2f\n', mean(settling_time_data(:,3)), std(settling_time_data(:,3)));
fprintf('ψ:    %7.2f  %7.2f\n', mean(settling_time_data(:,4)), std(settling_time_data(:,4)));

%% PSO Optimization Function
function [global_best, B] = optimize_pid_with_pso()
    % PSO Parameters
    nVar = 12; % 4 controllers × 3 parameters (Kp, Ki, Kd)
    VarMin = [2.0  0.01  0.1  0.1  0.001  0.1  0.1  0.001  0.1  0.1  0.001  0.1];    
    VarMax = [15   2.0   5    10   0.1    2    10   0.1    2    10   0.1    2]; 
    
    MaxIter = 100;
    nPop = 50;
    w = 0.7;
    w_damp = 0.99;
    c1 = 1.7;
    c2 = 1.7;
    
    % Initialize
    empty_particle.position = [];
    empty_particle.velocity = [];
    empty_particle.cost = [];
    empty_particle.best.position = [];
    empty_particle.best.cost = [];
    
    pop = repmat(empty_particle, nPop, 1);
    global_best.cost = inf;
    B = zeros(MaxIter, 1);
    
    % Initial population
    for i = 1:nPop
        pop(i).position = unifrnd(VarMin, VarMax);
        pop(i).velocity = zeros(1, nVar);
        pop(i).cost = pid_objective_function(pop(i).position);
        pop(i).best.position = pop(i).position;
        pop(i).best.cost = pop(i).cost;
        
        if pop(i).best.cost < global_best.cost
            global_best = pop(i).best;
        end
    end
    
    % PSO main loop
    for it = 1:MaxIter
        for i = 1:nPop
            % Update velocity
            pop(i).velocity = w * pop(i).velocity ...
                + c1 * rand(1,nVar) .* (pop(i).best.position - pop(i).position) ...
                + c2 * rand(1,nVar) .* (global_best.position - pop(i).position);
            
            % Update position
            pop(i).position = pop(i).position + pop(i).velocity;
            
            % Apply bounds
            pop(i).position = max(pop(i).position, VarMin);
            pop(i).position = min(pop(i).position, VarMax);
            
            % Evaluation
            pop(i).cost = pid_objective_function(pop(i).position);
            
            % Update personal best
            if pop(i).cost < pop(i).best.cost
                pop(i).best.position = pop(i).position;
                pop(i).best.cost = pop(i).cost;
                
                % Update global best
                if pop(i).best.cost < global_best.cost
                    global_best = pop(i).best;
                end
            end
        end
        
        % Store best cost
        B(it) = global_best.cost;
        
        % Display iteration info
        if mod(it,10) == 0
            fprintf('Iteration %d: Best Cost = %.4f\n', it, B(it));
        end
        
        % Damping inertia weight
        w = w * w_damp;
    end
end

%% PID Objective Function
function cost = pid_objective_function(ganancias)
    % System parameters
    m = 1.0; g = 9.81; 
    Ix = 0.1; Iy = 0.1; Iz = 0.2;
    
    % Initial conditions
    x0 = zeros(6,1); xdot0 = zeros(6,1);
    X0 = [x0; xdot0];
    tspan = [0 10];
    
    % Desired values
    z_des = 1; 
    phi_des = 0; 
    theta_des = 0; 
    psi_des = pi/4;
    
    % Extract gains
    Kp_z = ganancias(1); Ki_z = ganancias(2); Kd_z = ganancias(3);
    Kp_phi = ganancias(4); Ki_phi = ganancias(5); Kd_phi = ganancias(6);
    Kp_theta = ganancias(7); Ki_theta = ganancias(8); Kd_theta = ganancias(9);
    Kp_psi = ganancias(10); Ki_psi = ganancias(11); Kd_psi = ganancias(12);
    
    % Simulate system
    try
        [t, X] = ode45(@(t,X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
                      Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
                      Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
                      z_des, phi_des, theta_des, psi_des), tspan, X0);
        
        % Calculate errors
        error_z = abs(z_des - X(:,3));
        error_phi = abs(phi_des - X(:,4));
        error_theta = abs(theta_des - X(:,5));
        error_psi = abs(psi_des - X(:,6));
        
        % Overshoot penalty
        overshoot_penalty = 0;
        max_overshoot = max(0, max(X(:,3)) - z_des);
        if max_overshoot > 0
            overshoot_penalty = 10 * max_overshoot;
        end
        
        % Steady-state error
        if length(t) > 100
            steady_state_error = mean(error_z(end-100:end)) + ...
                                mean(error_phi(end-100:end)) + ...
                                mean(error_theta(end-100:end)) + ...
                                mean(error_psi(end-100:end));
        else
            steady_state_error = mean(error_z) + mean(error_phi) + ...
                                 mean(error_theta) + mean(error_psi);
        end
        
        % Combined cost (ITAE + penalties)
        cost = trapz(t, t.*error_z) + trapz(t, t.*error_phi) + ...
               trapz(t, t.*error_theta) + trapz(t, t.*error_psi) + ...
               overshoot_penalty + 10*steady_state_error;
           
    catch
        % If simulation fails, assign high cost
        cost = 1e4 + norm(ganancias);
    end
end

%% Quadrotor Dynamics
function dXdt = quadrotor_dynamics(~, X, m, g, Ix, Iy, Iz,...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
        z_des, phi_des, theta_des, psi_des)
    
    persistent integral_z integral_phi integral_theta integral_psi
    if isempty(integral_z)
        integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;
    end
    
    % Extract states
    pos = X(1:6);       % [x, y, z, ϕ, θ, ψ]
    vel = X(7:12);      % [dx, dy, dz, dϕ, dθ, dψ]
    
    % Calculate errors
    errores = [z_des - pos(3);    % Altitude error
              phi_des - pos(4);   % Roll error
              theta_des - pos(5); % Pitch error
              psi_des - pos(6)];  % Yaw error
    
    % Update integrals (with anti-windup)
    max_int = 10;
    integral_z = max(min(integral_z + errores(1), max_int), -max_int);   
    integral_phi = max(min(integral_phi + errores(2), max_int), -max_int); 
    integral_theta = max(min(integral_theta + errores(3), max_int), -max_int); 
    integral_psi = max(min(integral_psi + errores(4), max_int), -max_int);
    
    % PID Control
    U1 = Kp_z*errores(1) + Ki_z*integral_z + Kd_z*(-vel(3));     % Thrust
    U2 = Kp_phi*errores(2) + Ki_phi*integral_phi + Kd_phi*(-vel(4)); % Roll torque
    U3 = Kp_theta*errores(3) + Ki_theta*integral_theta + Kd_theta*(-vel(5)); % Pitch torque
    U4 = Kp_psi*errores(4) + Ki_psi*integral_psi + Kd_psi*(-vel(6));   % Yaw torque
    
    % Translational dynamics
    acc_lin = [...
        (cos(pos(4))*sin(pos(5))*cos(pos(6)) + sin(pos(4))*sin(pos(6)))*U1/m;
        (cos(pos(4))*sin(pos(5))*sin(pos(6)) - sin(pos(4))*cos(pos(6)))*U1/m;
        (cos(pos(4))*cos(pos(5))*U1/m) - g];
    
    % Rotational dynamics
    acc_ang = [...
        (U2 + (Iy - Iz)*vel(5)*vel(6))/Ix;
        (U3 + (Iz - Ix)*vel(4)*vel(6))/Iy;
        (U4 + (Ix - Iy)*vel(4)*vel(5))/Iz];
    
    % Derivative vector
    dXdt = [vel; acc_lin; acc_ang];
end

%% Settling Time Calculation Function
function settling_time = calculate_settling_time(t, y, y_final, tolerance)
    % Find when the response enters and stays within the tolerance band
    abs_error = abs(y - y_final);
    
    % Handle case where y_final is zero
    if abs(y_final) < eps
        tolerance_band = tolerance;
    else
        tolerance_band = tolerance * abs(y_final);
    end
    
    % Find all points within tolerance
    in_tolerance = abs_error <= tolerance_band;
    
    % Initialize
    settling_time = t(end); % Default to end of simulation if never settles
    
    % Find the first point where system stays within tolerance
    for i = 1:length(in_tolerance)
        if all(in_tolerance(i:end))
            settling_time = t(i);
            break;
        end
    end
    
    % Ensure scalar output
    settling_time = settling_time(1);
end