clear; clc; close all;

dt = 0.02;
T_total = 200;
t = 0:dt:T_total;

runwayLength = 3048;
acceleration = 1.08;
t_rotation_start = sqrt(2 * runwayLength / acceleration);
V_takeoff = acceleration * t_rotation_start;
rotation_duration = 10;
initial_pitch = deg2rad(15);
final_pitch = deg2rad(5);
climb_rate = 5;
t_liftoff = t_rotation_start + rotation_duration;
t_pitch_reduce_start = t_liftoff + 5;
t_pitch_reduce_end = t_pitch_reduce_start + 10;

X = zeros(size(t));
Y = zeros(size(t));
Z = zeros(size(t));
pitch_angle = zeros(size(t));
heading_angle = zeros(size(t));

psi_desired = deg2rad(45);
Kp = 0.3;
Ki = 0.05;
Kd = 0.1;
int_err = 0;
err_prev = 0;

for i = 2:length(t)
    current_time = t(i);
%     if current_time <= t_rotation_start
%         X(i) = 0.5 * acceleration * current_time^2;
%         Z(i) = 0;
%         pitch_angle(i) = 0;
%     elseif current_time <= t_liftoff
%         X(i) = X(i-1) + V_takeoff * dt;
%         pitch_angle(i) = ((current_time - t_rotation_start) / rotation_duration) * initial_pitch;
%         Z(i) = (current_time - t_rotation_start) * V_takeoff * sin(pitch_angle(i));
%     elseif current_time <= t_pitch_reduce_start
%         X(i) = X(i-1) + V_takeoff * dt;
%         pitch_angle(i) = initial_pitch;
%         Z(i) = Z(i-1) + climb_rate * dt;
%     elseif current_time <= t_pitch_reduce_end
%         X(i) = X(i-1) + V_takeoff * dt;
%         pitch_angle(i) = initial_pitch - ((current_time - t_pitch_reduce_start) / (t_pitch_reduce_end - t_pitch_reduce_start)) * (initial_pitch - final_pitch);
%         Z(i) = Z(i-1) + climb_rate * dt;

    if current_time <= t_rotation_start
        X(i) = 0.5 * acceleration * current_time^2;
        Z(i) = 0;
        pitch_angle(i) = 0;

    elseif current_time <= t_liftoff
        X(i) = X(i-1) + V_takeoff * dt;
        pitch_angle(i) = ((current_time - t_rotation_start) / rotation_duration) * initial_pitch;
        Z(i) = Z(i-1) + sin(pitch_angle(i)) * V_takeoff * dt;
    else
        X(i) = X(i-1) + V_takeoff * dt * cos(heading_angle(i-1));
        Y(i) = Y(i-1) + V_takeoff * dt * sin(heading_angle(i-1));
        pitch_angle(i) = initial_pitch;
        Z(i) = Z(i-1) + sin(pitch_angle(i)) * V_takeoff * dt;
    end
    heading_error = psi_desired - heading_angle(i-1);
    int_err = int_err + heading_error * dt;
    derr = (heading_error - err_prev) / dt;
    err_prev = heading_error;
    heading_angle(i) = heading_angle(i-1) + (Kp * heading_error + Ki * int_err + Kd * derr) * dt;
end

figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
title('3D Aircraft Takeoff Simulation');
view(3);
xlim([0 max(X)*1.05]);
ylim([-max(abs(Y))/3, max(abs(Y))/3]);
zlim([0, max(Z) * 1.2]);

aircraftModel = stlread('747.stl');
scaleFactor = 2;
vertices = aircraftModel.Points * scaleFactor;

Rx = makehgtform('zrotate', 5*pi/4);  
rotatedPoints = (Rx(1:3,1:3) * vertices')';
aircraft = patch('Faces', aircraftModel.ConnectivityList, 'Vertices', rotatedPoints, 'FaceColor', 'blue', 'EdgeColor', 'none');

hgt = hgtransform;
set(aircraft, 'Parent', hgt);
trail = plot3(nan, nan, nan, 'r-', 'LineWidth', 1.5);
pivot_offset = [-20, 0, -10];
T_offset = makehgtform('translate', pivot_offset);
T_offset_inv = makehgtform('translate', -pivot_offset);

for i = 1:length(t)
    T_translate = makehgtform('translate', [X(i), Y(i), Z(i)]);
    T_yaw = makehgtform('zrotate', heading_angle(i));  % Apply heading (turn)
    T_pitch = makehgtform('yrotate', -pitch_angle(i));  % Apply pitch (climb)

    % Ensure transformations are in the correct order
    T = T_translate * T_yaw * T_pitch;  

    set(hgt, 'Matrix', T);
    set(trail, 'XData', X(1:i), 'YData', Y(1:i), 'ZData', Z(1:i));
    pause(0.005);
end
