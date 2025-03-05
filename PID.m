% Physical parameters
m = 1.0; % Mass (kg)
g = 9.81; % Gravity (m/s^2)
Ix = 0.1; Iy = 0.1; Iz = 0.2; % Moments of inertia (kg*m^2)

% Initial conditions
x0 = [0; 0; 0; 0; 0; 0]; % [x, y, z, phi, theta, psi]
xdot0 = [0; 0; 0; 0; 0; 0]; % [dx, dy, dz, dphi, dtheta, dpsi]

% PID gains for Z, Roll, Pitch, Yaw
Kp_z = 10; Ki_z = 0.1; Kd_z = 5;
Kp_phi = 5; Ki_phi = 0.01; Kd_phi = 1;
Kp_theta = 5; Ki_theta = 0.01; Kd_theta = 1;
Kp_psi = 5; Ki_psi = 0.01; Kd_psi = 1;

% Time span
tspan = [0 10];

% Initial state vector
X0 = [x0; xdot0];

% ODE solver
[t, X] = ode45(@(t, X) quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz, ...
    Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi, ...
    Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi), tspan, X0);

% Plot altitude
figure;
plot(t, X(:, 3));
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Quadrotor Altitude');

function dXdt = quadrotor_dynamics(t, X, m, g, Ix, Iy, Iz, ...
        Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi, ...
        Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi)

    % Extract states
    x = X(1); y = X(2); z = X(3);
    phi = X(4); theta = X(5); psi = X(6);
    dx = X(7); dy = X(8); dz = X(9);
    dphi = X(10); dtheta = X(11); dpsi = X(12);

    % Desired states
    z_des = 1; % Desired altitude
    phi_des = 0; % Desired roll
    theta_des = 0; % Desired pitch
    psi_des = 0; % Desired yaw

    % PID control for Z
    U1 = Kp_z * (z_des - z) + Ki_z * integral_z + Kd_z * (0 - dz);

    % PID control for Roll, Pitch, Yaw
    U2 = Kp_phi * (phi_des - phi) + Ki_phi * integral_phi + Kd_phi * (0 - dphi);
    U3 = Kp_theta * (theta_des - theta) + Ki_theta * integral_theta + Kd_theta * (0 - dtheta);
    U4 = Kp_psi * (psi_des - psi) + Ki_psi * integral_psi + Kd_psi * (0 - dpsi);

    % Equations of motion
    ddx = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * U1 / m;
    ddy = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * U1 / m;
    ddz = (cos(phi)*cos(theta)) * U1 / m - g;

    ddphi = (U2 + (Iy - Iz)*dtheta*dpsi) / Ix;
    ddtheta = (U3 + (Iz - Ix)*dphi*dpsi) / Iy;
    ddpsi = (U4 + (Ix - Iy)*dphi*dtheta) / Iz;

    % Return derivatives
    dXdt = [dx; dy; dz; dphi; dtheta; dpsi; ddx; ddy; ddz; ddphi; ddtheta; ddpsi];
end

