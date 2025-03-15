clear; clc; close all;

% Simulation setup

% Time setup
dt = 0.02;
T_total = 200;
t = 0:dt:T_total;

% Aircraft and runway parameters
runwayLength = 3048;
acceleration = 1.08;
t_rotation_start = sqrt(2 * runwayLength / acceleration);
V_takeoff = acceleration * t_rotation_start;
rotation_duration = 20;
initial_pitch = deg2rad(15);
climb_rate = 5;

% Heading control parameters
psi_desired = deg2rad(45);  % Desired heading (45 degrees)
Kp = 0.3; Ki = 0.05; Kd = 0.1; % PID gains
int_err = 0;
err_prev = 0;

% Position and angle initialization
X = zeros(size(t));
Y = zeros(size(t));
Z = zeros(size(t));
pitch_angle = zeros(size(t));
heading_angle = zeros(size(t));

% Takeoff and flight dynamics loop
for i = 2:length(t)
    current_time = t(i);

    % Takeoff roll
    if current_time <= t_rotation_start
        X(i) = 0.5 * acceleration * current_time^2;
        Z(i) = 0;
        pitch_angle(i) = 0;

    % Rotation
    elseif current_time <= t_rotation_start + rotation_duration
        X(i) = X(i-1) + V_takeoff * dt;
        pitch_angle(i) = ((current_time - t_rotation_start) / rotation_duration) * initial_pitch;
        Z(i) = Z(i-1) + sin(pitch_angle(i)) * V_takeoff * dt;

    % Climb phase
    else
        X(i) = X(i-1) + V_takeoff * dt * cos(heading_angle(i-1));
        Y(i) = Y(i-1) + V_takeoff * dt * sin(heading_angle(i-1));
        pitch_angle(i) = initial_pitch;
        Z(i) = Z(i-1) + sin(pitch_angle(i)) * V_takeoff * dt;

        % PID Heading Control
        heading_error = psi_desired - heading_angle(i-1);
        int_err = int_err + heading_error * dt;
        derr = (heading_error - err_prev) / dt;
        err_prev = heading_error;

        heading_angle(i) = heading_angle(i-1) + (Kp * heading_error + Ki * int_err + Kd * derr) * dt;
    end
end

% Visualization setup
figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
title('3D Aircraft Flight Path Simulation');
view(3);
xlim([0 max(X)*1.05]);
ylim([-max(abs(Y)), max(abs(Y))]);
maxAltitude = max(Z) + 5;
zlim([0, maxAltitude * 1.2]);

% Load aircraft model
aircraftModel = stlread('747.stl');
scaleFactor = 1;
vertices = aircraftModel.Points * scaleFactor;
Rx = makehgtform('zrotate', -pi/2);
rotatedPoints = (Rx(1:3,1:3) * vertices')';
aircraft = patch('Faces', aircraftModel.ConnectivityList, 'Vertices', rotatedPoints, 'FaceColor', 'blue', 'EdgeColor', 'none');

hgt = hgtransform;
set(aircraft, 'Parent', hgt);
trail = plot3(nan, nan, nan, 'r-', 'LineWidth', 1.5);

% Aircraft animation loop
for i = 1:length(t)
    T_translate = makehgtform('translate', [X(i), Y(i), Z(i) + 5]);
    T_yaw = makehgtform('zrotate', heading_angle(i));
    T_pitch = makehgtform('yrotate', -pitch_angle(i));
    T = T_translate * T_yaw * T_pitch;
    set(hgt, 'Matrix', T);
    set(trail, 'XData', X(1:i), 'YData', Y(1:i), 'ZData', Z(1:i) + 5);
    pause(0.005);
end
