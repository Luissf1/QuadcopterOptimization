function RMSEZN()
    % --- Parameters and initial conditions ---
    m = 1.0;          % Mass 
    g = 9.81;         % Gravity 
    Ix = 0.1; Iy = 0.1; Iz = 0.2;  % Moments of inertia 

    x0 = [0; 0; 0; 0; 0; 0];       % Initial position [x, y, z, roll, pitch, yaw]
    xdot0 = [0; 0; 0; 0; 0; 0];    % Initial velocities
    X0 = [x0; xdot0];              % Full initial state

    % --- Simulation time ---
    tspan = [0 20]; % Simulation from 0 to 20 seconds

    % --- Desired flight conditions (same as PSO PID code) ---
    combinaciones_deseadas = [...
        1.0,  0.0,   0.0,    0.0;
        1.5,  0.1,  -0.1,    0.0;
        2.0, -0.2,   0.2,    0.0;
        1.0,  0.0,   0.0,    pi/4;
        0.5, -0.1,  -0.1,   -pi/6];

    % Initialize RMSE storage
    RMSE_results = zeros(size(combinaciones_deseadas, 1), 1);

    % Loop through each flight condition
    for i = 1:size(combinaciones_deseadas, 1)
        z_des = combinaciones_deseadas(i, 1);
        phi_des = combinaciones_deseadas(i, 2);
        theta_des = combinaciones_deseadas(i, 3);
        psi_des = combinaciones_deseadas(i, 4);

        fprintf('\n--- Flight %d: z=%.1f, phi=%.1f, theta=%.1f, psi=%.1f ---\n', ...
                i, z_des, phi_des, theta_des, psi_des);

        % --- Step 1: Find Ku and Pu for the current flight condition ---
        Ki_test = 0; 
        Kd_test = 0;
        Kp_values = 0:0.5:100;  % Finer resolution for Ku detection
        Ku_z = NaN; 
        Pu_z = NaN;

        for Kp_test = Kp_values
            % Reset integrals
            clear global integral_z integral_phi integral_theta integral_psi;
            global integral_z integral_phi integral_theta integral_psi;
            integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;

            % Simulate with current Kp
            [t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
                Kp_test, Ki_test, Kd_test, 0,0,0, 0,0,0, 0,0,0,...
                z_des, phi_des, theta_des, psi_des), tspan, X0);

            % Check for sustained oscillations
            z = X(:,3);
            z_error = z - z_des;
            zero_crossings = find(diff(sign(z_error)));

            if length(zero_crossings) >= 4  % At least 2 full oscillations
                Pu_z = mean(diff(t(zero_crossings(1:4)))); % Period from first 2 oscillations
                Ku_z = Kp_test;
                fprintf('  Found Ku = %.2f, Pu = %.2f\n', Ku_z, Pu_z);
                break;
            end
        end

        % --- Step 2: Calculate PID gains (with stability checks) ---
        if isnan(Ku_z) || isnan(Pu_z)
            fprintf('  Warning: No oscillations detected. Using backup gains.\n');
            Kp_z = 5.0;  % Default safe gains
            Ki_z = 0.5;
            Kd_z = 0.1;
        else
            % Conservative Ziegler-Nichols
            Kp_z = 0.6 * Ku_z;
            Ki_z = 1.2 * Kp_z / Pu_z;  % Reduced integral
            Kd_z = Kp_z * Pu_z / 12;    % Reduced derivative
        end

        % Attitude gains (simplified)
        Kp_phi = Kp_z;    Ki_phi = Ki_z;    Kd_phi = Kd_z;
        Kp_theta = Kp_z;  Ki_theta = Ki_z;  Kd_theta = Kd_z;
        Kp_psi = Kp_z;    Ki_psi = Ki_z;    Kd_psi = Kd_z;

        % --- Step 3: Simulate with tuned PID ---
        clear global integral_z integral_phi integral_theta integral_psi;
        global integral_z integral_phi integral_theta integral_psi;
        integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;

        try
            [t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
                Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
                Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
                z_des, phi_des, theta_des, psi_des), tspan, X0);

            % --- Step 4: Calculate RMSE ---
            z = X(:,3);
            error_z = z_des - z;
            RMSE = sqrt(mean(error_z.^2));
            RMSE_results(i) = RMSE;

            % Plot response
            figure(i);
            plot(t, z, 'b', t, z_des*ones(size(t)), 'r--');
            title(sprintf('Flight %d: z Response (RMSE=%.4f)', i, RMSE));
            xlabel('Time (s)'); ylabel('z (m)'); grid on;
            legend('Actual', 'Desired');

        catch
            fprintf('  Simulation crashed! Assigning RMSE = Inf.\n');
            RMSE_results(i) = Inf;
        end
    end

    % --- Display RMSE results ---
    fprintf('\n=== Final RMSE Results ===\n');
    for i = 1:size(combinaciones_deseadas, 1)
        fprintf('Flight %d (z=%.1f, phi=%.1f, theta=%.1f, psi=%.1f): RMSE = %.4f\n', ...
            i, combinaciones_deseadas(i, 1), combinaciones_deseadas(i, 2), ...
            combinaciones_deseadas(i, 3), combinaciones_deseadas(i, 4), RMSE_results(i));
    end
end

% ========== Quadrotor Dynamics Function ==========
function dXdt = quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
        z_des, phi_des, theta_des, psi_des)

    global integral_z integral_phi integral_theta integral_psi;
    
    % Anti-windup: Limit integral terms
    max_integral = 10;
    integral_z = max(min(integral_z, max_integral), -max_integral);
    integral_phi = max(min(integral_phi, max_integral), -max_integral);
    integral_theta = max(min(integral_theta, max_integral), -max_integral);
    integral_psi = max(min(integral_psi, max_integral), -max_integral);
    
    % Extract states
    pos = X(1:6);   % [x, y, z, ϕ, θ, ψ]
    vel = X(7:12);  % Velocity derivatives
    
    % Errors
    errors = [z_des - pos(3);    % Altitude error
              phi_des - pos(4);   % Roll error
              theta_des - pos(5); % Pitch error
              psi_des - pos(6)]; % Yaw error
    
    % PID Control
    U1 = Kp_z*errors(1) + Ki_z*integral_z + Kd_z*(-vel(3));  % Thrust
    U2 = Kp_phi*errors(2) + Ki_phi*integral_phi + Kd_phi*(-vel(4)); % Roll torque
    U3 = Kp_theta*errors(3) + Ki_theta*integral_theta + Kd_theta*(-vel(5)); % Pitch torque
    U4 = Kp_psi*errors(4) + Ki_psi*integral_psi + Kd_psi*(-vel(6)); % Yaw torque
    
    % Translational dynamics
    acc_lin = [...
        (cos(pos(4))*sin(pos(5))*cos(pos(6)) + sin(pos(4))*sin(pos(6))) * U1/m;
        (cos(pos(4))*sin(pos(5))*sin(pos(6)) - sin(pos(4))*cos(pos(6))) * U1/m;
        (cos(pos(4))*cos(pos(5)) * U1/m) - g];
    
    % Rotational dynamics
    acc_ang = [...
        (U2 + (Iy - Iz)*vel(5)*vel(6)) / Ix;
        (U3 + (Iz - Ix)*vel(4)*vel(6)) / Iy;
        (U4 + (Ix - Iy)*vel(4)*vel(5)) / Iz];
    
    % State derivatives
    dXdt = [vel; acc_lin; acc_ang];
end