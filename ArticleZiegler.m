% --- Parameters and initial conditions ---
m = 1.0;          % Mass 
g = 9.81;         % Gravity 
Ix = 0.1; Iy = 0.1; Iz = 0.2;  % Moments of inertia 

x0 = [0; 0; 0; 0; 0; 0];       % Initial position [x, y, z, roll, pitch, yaw]
xdot0 = [0; 0; 0; 0; 0; 0];    % Initial velocities

X0 = [x0; xdot0];              % Complete initial state

% Simulation time
tspan = [0 20];

% Desired values
z_des = 1; phi_des = 0; theta_des = 0; psi_des = 0;

% --- STEP 1: Find Critical Gain Ku and Oscillation Period Pu for each channel ---
% This is done by increasing Kp while Ki and Kd are 0, until output oscillates with constant amplitude.

% Initialize variables to store Ku and Pu for all channels
Ku = struct('z', NaN, 'phi', NaN, 'theta', NaN, 'psi', NaN);
Pu = struct('z', NaN, 'phi', NaN, 'theta', NaN, 'psi', NaN);

% Test values
Ki_test = 0; 
Kd_test = 0;
Kp_values = 0:1:50;  % Adjust according to your system

% Find Ku and Pu for each channel
channels = {'z', 'phi', 'theta', 'psi'};
for i = 1:length(channels)
    channel = channels{i};
    for Kp_test = Kp_values
        % Reset integral variables
        global integral_z integral_phi integral_theta integral_psi;
        integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;

        % Simulate with current Kp_test
        [t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
            strcmp(channel,'z')*Kp_test, 0, 0,...
            strcmp(channel,'phi')*Kp_test, 0, 0,...
            strcmp(channel,'theta')*Kp_test, 0, 0,...
            strcmp(channel,'psi')*Kp_test, 0, 0,...
            z_des, phi_des, theta_des, psi_des), tspan, X0);

        % Extract output signal
        if strcmp(channel, 'z')
            output = X(:,3);
            desired = z_des;
        elseif strcmp(channel, 'phi')
            output = X(:,4);
            desired = phi_des;
        elseif strcmp(channel, 'theta')
            output = X(:,5);
            desired = theta_des;
        else % psi
            output = X(:,6);
            desired = psi_des;
        end

        % Check for sustained oscillations
        error = output - desired;
        zero_crossings = find(diff(sign(error)));
        
        if length(zero_crossings) > 5  % If more than 5 crossings, likely sustained oscillation
            % Calculate period Pu as average of differences between crossings
            Pu.(channel) = mean(diff(t(zero_crossings)));
            Ku.(channel) = Kp_test;
            fprintf('%s: Ku = %.2f, Pu = %.2f\n', channel, Ku.(channel), Pu.(channel));
            break;
        end
    end
end

% --- STEP 2: Calculate PID parameters using Ziegler-Nichols formulas ---
% Classic ZN PID controller formulas:
% Kp = 0.6 * Ku
% Ki = 2 * Kp / Pu
% Kd = Kp * Pu / 8

% For altitude (z)
Kp_z = 0.6 * Ku.z;
Ki_z = 2 * Kp_z / Pu.z;
Kd_z = Kp_z * Pu.z / 8;

% For attitude (roll, pitch, yaw)
Kp_phi = 0.6 * Ku.phi;    Ki_phi = 2 * Kp_phi / Pu.phi;    Kd_phi = Kp_phi * Pu.phi / 8;
Kp_theta = 0.6 * Ku.theta; Ki_theta = 2 * Kp_theta / Pu.theta; Kd_theta = Kp_theta * Pu.theta / 8;
Kp_psi = 0.6 * Ku.psi;    Ki_psi = 2 * Kp_psi / Pu.psi;    Kd_psi = Kp_psi * Pu.psi / 8;

% Display PID parameters
fprintf('\n--- Ziegler-Nichols PID Parameters ---\n');
fprintf('Altitude (z): Kp=%.4f, Ki=%.4f, Kd=%.4f\n', Kp_z, Ki_z, Kd_z);
fprintf('Roll (phi): Kp=%.4f, Ki=%.4f, Kd=%.4f\n', Kp_phi, Ki_phi, Kd_phi);
fprintf('Pitch (theta): Kp=%.4f, Ki=%.4f, Kd=%.4f\n', Kp_theta, Ki_theta, Kd_theta);
fprintf('Yaw (psi): Kp=%.4f, Ki=%.4f, Kd=%.4f\n', Kp_psi, Ki_psi, Kd_psi);

% --- STEP 3: Simulate with obtained PID parameters ---
global integral_z integral_phi integral_theta integral_psi;
integral_z = 0; integral_phi = 0; integral_theta = 0; integral_psi = 0;

[t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
    Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
    Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
    z_des, phi_des, theta_des, psi_des), tspan, X0);

% --- STEP 4: Performance Metrics Calculation ---
% Initialize metrics structure
metrics = struct();

% For each channel
for i = 1:length(channels)
    channel = channels{i};
    
    % Extract data
    if strcmp(channel, 'z')
        output = X(:,3);
        desired = z_des;
    elseif strcmp(channel, 'phi')
        output = X(:,4);
        desired = phi_des;
    elseif strcmp(channel, 'theta')
        output = X(:,5);
        desired = theta_des;
    else % psi
        output = X(:,6);
        desired = psi_des;
    end
    
    % Calculate metrics
    error = desired - output;
    
    % Overshoot (%)
    if desired ~= 0
        overshoot = (max(output) - desired) / desired * 100;
    else
        overshoot = max(abs(output)) * 100; % For zero desired value
    end
    
    % MSE (Mean Squared Error)
    mse = mean(error.^2);
    
    % MAE (Mean Absolute Error)
    mae = mean(abs(error));
    
    % ISE (Integral of Squared Error)
    ise = trapz(t, error.^2);
    
    % IAE (Integral of Absolute Error)
    iae = trapz(t, abs(error));
    
    % Settling time (±5%)
    tolerance = 0.05 * abs(desired);
    if desired == 0
        tolerance = 0.05 * max(abs(output)); % Special case for zero desired
    end
    
    idx_settle = find(abs(output - desired) > tolerance, 1, 'last');
    if isempty(idx_settle)
        settling_time = 0;
    else
        settling_time = t(idx_settle);
    end
    
    % Rise time (10% to 90%)
    if desired ~= 0
        rise_start = desired * 0.1;
        rise_end = desired * 0.9;
    else
        % For zero desired, use peak value as reference
        peak_val = max(abs(output));
        rise_start = peak_val * 0.1;
        rise_end = peak_val * 0.9;
    end
    
    try
        t_rise_start = t(find(output >= rise_start, 1));
        t_rise_end = t(find(output >= rise_end, 1));
        rise_time = t_rise_end - t_rise_start;
    catch
        rise_time = NaN; % No complete rise detected
    end
    
    % Store metrics
    metrics.(channel) = struct(...
        'Kp', eval(['Kp_' channel]), ...
        'Ki', eval(['Ki_' channel]), ...
        'Kd', eval(['Kd_' channel]), ...
        'Overshoot', overshoot, ...
        'MSE', mse, ...
        'MAE', mae, ...
        'ISE', ise, ...
        'IAE', iae, ...
        'SettlingTime', settling_time, ...
        'RiseTime', rise_time);
end

% Display metrics
fprintf('\n--- Performance Metrics ---\n');
for i = 1:length(channels)
    channel = channels{i};
    m = metrics.(channel);
    fprintf('\n%s:\n', upper(channel));
    fprintf('Kp=%.4f, Ki=%.4f, Kd=%.4f\n', m.Kp, m.Ki, m.Kd);
    fprintf('Overshoot: %.2f%%\n', m.Overshoot);
    fprintf('MSE: %.5f, MAE: %.5f\n', m.MSE, m.MAE);
    fprintf('ISE: %.5f, IAE: %.5f\n', m.ISE, m.IAE);
    fprintf('Settling Time: %.2f s, Rise Time: %.2f s\n', m.SettlingTime, m.RiseTime);
end

% --- STEP 5: Generate Excel File with Results ---
% Prepare data for Excel
header = {'Channel', 'Kp', 'Ki', 'Kd', 'Overshoot%', 'MSE', 'MAE', 'ISE', 'IAE', 'SettlingTime', 'RiseTime'};
data = cell(length(channels), length(header));

for i = 1:length(channels)
    channel = channels{i};
    m = metrics.(channel);
    data{i,1} = upper(channel);
    data{i,2} = m.Kp;
    data{i,3} = m.Ki;
    data{i,4} = m.Kd;
    data{i,5} = m.Overshoot;
    data{i,6} = m.MSE;
    data{i,7} = m.MAE;
    data{i,8} = m.ISE;
    data{i,9} = m.IAE;
    data{i,10} = m.SettlingTime;
    data{i,11} = m.RiseTime;
end

% Write to Excel
filename = 'Quadrotor_PID_Performance.xlsx';
writecell(header, filename, 'Sheet', 'Performance', 'Range', 'A1');
writecell(data, filename, 'Sheet', 'Performance', 'Range', 'A2');

fprintf('\nResults saved to %s\n', filename);

% --- STEP 6: Enhanced Plotting ---
% Create figure with better formatting for publication
figure('Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);

% Altitude plot
subplot(2,2,1);
plot(t, X(:,3), 'b', 'LineWidth', 1.5);
hold on;
plot(t, z_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
title('Altitude Control', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 10);
ylabel('Altitude (m)', 'FontSize', 10);
legend('Actual', 'Desired', 'Location', 'best');
grid on;
box on;

% Add performance metrics to plot
text(0.6*max(t), 0.8*z_des, sprintf('Overshoot: %.1f%%\nSettling Time: %.2f s', metrics.z.Overshoot, metrics.z.SettlingTime),...
    'FontSize', 9, 'BackgroundColor', 'white');

% Roll plot
subplot(2,2,2);
plot(t, X(:,4)*180/pi, 'b', 'LineWidth', 1.5); % Convert to degrees
hold on;
plot(t, phi_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
title('Roll Angle Control', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 10);
ylabel('Roll Angle (deg)', 'FontSize', 10);
legend('Actual', 'Desired', 'Location', 'best');
grid on;
box on;
text(0.6*max(t), 0.8*max(X(:,4)*180/pi), sprintf('Overshoot: %.1f%%\nSettling Time: %.2f s', metrics.phi.Overshoot, metrics.phi.SettlingTime),...
    'FontSize', 9, 'BackgroundColor', 'white');

% Pitch plot
subplot(2,2,3);
plot(t, X(:,5)*180/pi, 'b', 'LineWidth', 1.5); % Convert to degrees
hold on;
plot(t, theta_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
title('Pitch Angle Control', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 10);
ylabel('Pitch Angle (deg)', 'FontSize', 10);
legend('Actual', 'Desired', 'Location', 'best');
grid on;
box on;
text(0.6*max(t), 0.8*max(X(:,5)*180/pi), sprintf('Overshoot: %.1f%%\nSettling Time: %.2f s', metrics.theta.Overshoot, metrics.theta.SettlingTime),...
    'FontSize', 9, 'BackgroundColor', 'white');

% Yaw plot
subplot(2,2,4);
plot(t, X(:,6)*180/pi, 'b', 'LineWidth', 1.5); % Convert to degrees
hold on;
plot(t, psi_des*ones(size(t)), 'r--', 'LineWidth', 1.5);
title('Yaw Angle Control', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 10);
ylabel('Yaw Angle (deg)', 'FontSize', 10);
legend('Actual', 'Desired', 'Location', 'best');
grid on;
box on;
text(0.6*max(t), 0.8*max(X(:,6)*180/pi), sprintf('Overshoot: %.1f%%\nSettling Time: %.2f s', metrics.psi.Overshoot, metrics.psi.SettlingTime),...
    'FontSize', 9, 'BackgroundColor', 'white');

% Save figure
saveas(gcf, 'Quadrotor_PID_Performance.png');
fprintf('Performance plots saved to Quadrotor_PID_Performance.png\n');

% --- Dynamics Function (same as before) ---
function dXdt = quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz,...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,...
        z_des, phi_des, theta_des, psi_des)

    global integral_z integral_phi integral_theta integral_psi;
    
    % Extract states
    pos = X(1:6);       % [x, y, z, ϕ, θ, ψ]
    vel = X(7:12);      % [dx, dy, dz, dϕ, dθ, dψ]
    
    % Error calculation
    errors = [z_des - pos(3);    % Altitude error
              phi_des - pos(4);   % Roll error
              theta_des - pos(5); % Pitch error
              psi_des - pos(6)];  % Yaw error
    
    % Update integrals
    integral_z = integral_z + errors(1);
    integral_phi = integral_phi + errors(2);
    integral_theta = integral_theta + errors(3);
    integral_psi = integral_psi + errors(4);
    
    % PID control
    U1 = Kp_z*errors(1) + Ki_z*integral_z + Kd_z*(-vel(3));
    U2 = Kp_phi*errors(2) + Ki_phi*integral_phi + Kd_phi*(-vel(4));
    U3 = Kp_theta*errors(3) + Ki_theta*integral_theta + Kd_theta*(-vel(5));
    U4 = Kp_psi*errors(4) + Ki_psi*integral_psi + Kd_psi*(-vel(6));
    
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