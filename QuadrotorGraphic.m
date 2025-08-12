% Define Euler angles (ZYX convention)
psi = pi/4;    % Yaw (radians)
theta = pi/6;  % Pitch (radians)
phi = pi/8;    % Roll (radians)

%% Figure 1: Inertial Frame (ℰ)
figure;
hold on; 
axis equal; 
grid on;
view(30, 30);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Inertial Frame (ℰ)');

% Draw inertial frame axes
quiver3(0,0,0, 1,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); 
text(1.1,0,0, 'x_E', 'FontSize', 12, 'Color', 'r');
quiver3(0,0,0, 0,1,0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); 
text(0,1.1,0, 'y_E', 'FontSize', 12, 'Color', 'g');
quiver3(0,0,0, 0,0,1, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5); 
text(0,0,1.1, 'z_E', 'FontSize', 12, 'Color', 'b');

% Save inertial frame figure
exportgraphics(gcf, 'inertial_frame.pdf', 'ContentType', 'vector');

%% Figure 2: Body Frame (ℬ) with Euler Angles
figure;
hold on; 
axis equal; 
grid on;
view(30, 30);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Body Frame (ℬ) with Euler Angles');

% Get rotation matrix (ZYX convention)
R = eul2rotm([psi, theta, phi], 'ZYX'); 

% Draw body frame axes (rotated)
quiver3(0,0,0, R(1,1),R(2,1),R(3,1), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
text(1.1*R(1,1), 1.1*R(2,1), 1.1*R(3,1), 'x_B', 'FontSize', 12, 'Color', 'r');
quiver3(0,0,0, R(1,2),R(2,2),R(3,2), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
text(1.1*R(1,2), 1.1*R(2,2), 1.1*R(3,2), 'y_B', 'FontSize', 12, 'Color', 'g');
quiver3(0,0,0, R(1,3),R(2,3),R(3,3), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
text(1.1*R(1,3), 1.1*R(2,3), 1.1*R(3,3), 'z_B', 'FontSize', 12, 'Color', 'b');

% Add Euler angle annotations
text(0, 0, 1.3, sprintf('\\psi = %.0f°', rad2deg(psi)), 'HorizontalAlignment', 'center');
text(0, 1.3, 0, sprintf('\\theta = %.0f°', rad2deg(theta)), 'HorizontalAlignment', 'center');
text(1.3, 0, 0, sprintf('\\phi = %.0f°', rad2deg(phi)), 'HorizontalAlignment', 'center');

% Add quadrotor silhouette
arm_length = 0.2;
plot3([-arm_length, arm_length], [0, 0], [0, 0], 'k', 'LineWidth', 2); % X-arm
plot3([0, 0], [-arm_length, arm_length], [0, 0], 'k', 'LineWidth', 2); % Y-arm

% Save body frame figure
exportgraphics(gcf, 'body_frame.pdf', 'ContentType', 'vector');