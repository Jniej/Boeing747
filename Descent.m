clear; clc; close all;

dt = 0.02;
T_total = 100;
t = 0:dt:T_total;

initial_altitude = 10000;
target_altitude = 1000;
V_descent = 200;
desired_pitch = deg2rad(-3);  % Target pitch angle of -3 degrees

X = zeros(size(t));
Z = ones(size(t)) * initial_altitude;
pitch_angle = zeros(size(t));

% PID Gains
Kp = 0.05;  
Ki = 0.008; 
Kd = 0.005;  
error_sum = 0;
previous_error = 0;
integral_limit = 5;

for i = 2:length(t)
    altitude_error = target_altitude - Z(i-1);
    
    % PID calculations
    error_sum = error_sum + altitude_error * dt;
    error_sum = max(min(error_sum, integral_limit), -integral_limit);  % Limit integral windup
    error_derivative = (altitude_error - previous_error) / dt;
    
    % Compute pitch angle adjustment
    pitch_correction = Kp * altitude_error + Ki * error_sum + Kd * error_derivative;
    pitch_angle(i) = desired_pitch + pitch_correction;
    
    % Constrain pitch angle to hover around -3 degrees
    pitch_angle(i) = max(min(pitch_angle(i), deg2rad(0)), deg2rad(-6)); 
    
    previous_error = altitude_error;
    
    % Update aircraft position
    Vz = V_descent * sin(pitch_angle(i));
    X(i) = X(i-1) + V_descent * dt;
    Z(i) = Z(i-1) + Vz * dt;
end

% Plot PID-controlled descent angle
figure;
plot(t, rad2deg(pitch_angle), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Pitch Angle (degrees)');
title('PID-Controlled Descent Angle (Centered Around -3°)');
grid on;
legend('Pitch Angle');

% 3D Aircraft Descent Simulation
figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
title('3D Aircraft Descent Simulation with PID Control');
view(3);
xlim([0 max(X)*1.05]);
ylim([-200, 200]);
zlim([0, initial_altitude * 1.2]);

aircraftModel = stlread('747.stl');
scaleFactor = 3;
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
    T_translate = makehgtform('translate', [X(i) 0 Z(i)]);
    T_rotate = makehgtform('yrotate', -pitch_angle(i));
    T = T_translate * T_offset_inv * T_rotate * T_offset;
    set(hgt, 'Matrix', T);
    set(trail, 'XData', X(1:i), 'YData', zeros(1,i), 'ZData', Z(1:i));
    pause(0.005);
end
