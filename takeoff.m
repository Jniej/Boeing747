clear; clc; close all;

dt = 0.02;
T_total = 100;
t = 0:dt:T_total;

runwayLength = 3048;
acceleration = 1.08;
% acceleration = 3;
t_rotation_start = sqrt(2 * runwayLength / acceleration);
V_takeoff = acceleration * t_rotation_start;
rotation_duration = 20;
initial_pitch = deg2rad(15);
final_pitch = deg2rad(5);
climb_rate = 5;
t_liftoff = t_rotation_start + rotation_duration;
t_pitch_reduce_start = t_liftoff + 10;
t_pitch_reduce_end = t_pitch_reduce_start + 25;

X = zeros(size(t));
Z = zeros(size(t));
pitch_angle = zeros(size(t));

% for i = 2:length(t)
%     current_time = t(i);
%     if current_time <= t_rotation_start
%         X(i) = 0.5 * acceleration * current_time^2;
%         Z(i) = 0;
%         pitch_angle(i) = 0;
%     elseif current_time <= t_liftoff
%         X(i) = X(i-1) + V_takeoff * dt;
%         pitch_angle(i) = ((current_time - t_rotation_start) / rotation_duration) * initial_pitch;
%         %pitch_angle(i) = ((current_time - t_rotation_start) / rotation_duration) * final_pitch;
%         Z(i) = (current_time - t_rotation_start) * V_takeoff * sin(pitch_angle(i));
% %     elseif current_time <= t_pitch_reduce_start
% %         X(i) = X(i-1) + V_takeoff * dt;
% %         pitch_angle(i) = initial_pitch;
% %         Z(i) = Z(i-1) + climb_rate * dt;
% %     elseif current_time <= t_pitch_reduce_end
% %         X(i) = X(i-1) + V_takeoff * dt;
% %         pitch_angle(i) = initial_pitch - ((current_time - t_pitch_reduce_start) / (t_pitch_reduce_end - t_pitch_reduce_start)) * (initial_pitch - final_pitch);
% %         Z(i) = Z(i-1) + climb_rate * dt;
%     else
%         X(i) = X(i-1) + V_takeoff * dt;
%         pitch_angle(i) = initial_pitch;
%         Z(i) = Z(i-1) + climb_rate * dt;
%     end
% end

for i = 2:length(t)
    current_time = t(i);

    % Takeoff roll — accelerate along the runway
    if current_time <= t_rotation_start
        X(i) = 0.5 * acceleration * current_time^2;
        Z(i) = 0;
        pitch_angle(i) = 0;

    % Rotation — gradually pitch up to initial pitch
    elseif current_time <= t_liftoff
        X(i) = X(i-1) + V_takeoff * dt;
        pitch_angle(i) = ((current_time - t_rotation_start) / rotation_duration) * initial_pitch;
%         Z(i) = (current_time - t_rotation_start) * V_takeoff * sin(pitch_angle(i));
        Z(i) = Z(i-1) + sin(pitch_angle(i)) * V_takeoff * dt;

    % **Climb phase** — maintain initial pitch and steady climb
    else
        X(i) = X(i-1) + V_takeoff * dt;
        pitch_angle(i) = initial_pitch;  % Keep pitch constant at initial value
        Z(i) = Z(i-1) + sin(pitch_angle(i)) * V_takeoff * dt;
    end
end

figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
title('3D Aircraft Takeoff Simulation');
view(3);
xlim([0 max(X)*1.05]);
ylim([-200, 200]);
ground_clearance = 5;
maxAltitude = max(Z) + ground_clearance;
if maxAltitude < 50
    maxAltitude = 50;
end
zlim([0, maxAltitude * 1.2]);

aircraftModel = stlread('747.stl');
scaleFactor = 1;
vertices = aircraftModel.Points * scaleFactor;
Rx = makehgtform('zrotate', -pi/2);
rotatedPoints = (Rx(1:3,1:3) * vertices')';
aircraft = patch('Faces', aircraftModel.ConnectivityList, 'Vertices', rotatedPoints, 'FaceColor', 'blue', 'EdgeColor', 'none');

hgt = hgtransform;
set(aircraft, 'Parent', hgt);
trail = plot3(nan, nan, nan, 'r-', 'LineWidth', 1.5);

pivot_offset = [-20, 0, -10];
T_offset = makehgtform('translate', pivot_offset);
T_offset_inv = makehgtform('translate', -pivot_offset);

for i = 1:length(t)
    T_translate = makehgtform('translate', [X(i) 0 Z(i) + ground_clearance]);
    T_rotate = makehgtform('yrotate', -pitch_angle(i));
    T = T_translate * T_offset_inv * T_rotate * T_offset;
    set(hgt, 'Matrix', T);
    set(trail, 'XData', X(1:i), 'YData', zeros(1,i), 'ZData', Z(1:i) + ground_clearance);
    pause(0.005);
end
