clear; clc; close all;

dt = 0.02;
T_total = 200;
t = 0:dt:T_total;

initial_altitude = 10000;
target_altitude = 1000;
V_target = 200;  
leveling_threshold = 1500;  
zero_roll_angle = 0;  
max_throttle = 1.0;
min_throttle = 0.2;
initial_throttle = 0.7;

bank_angle_max = deg2rad(25);
bank_threshold_alt = 5000;

X = zeros(size(t));
Y = zeros(size(t));
Z = ones(size(t)) * initial_altitude;
V = ones(size(t)) * V_target;
throttle = ones(size(t)) * initial_throttle;
pitch_angle = zeros(size(t));
roll_angle = zeros(size(t));
roll_target = zeros(size(t));

Kp_speed = 0.02;  
Ki_speed = 0.005;  
Kd_speed = 0.002;  
error_sum_speed = 0;
previous_error_speed = 0;
integral_limit_speed = 5;

Kp_roll = 0.5;  
Ki_roll = 0.1;
Kd_roll = 0.02;
error_sum_roll = 0;
previous_error_roll = 0;
integral_limit_roll = 3;

Kp_pitch = 1;
Ki_pitch = 0.02;
Kd_pitch = 0.4;
error_sum_pitch = 0;
previous_error_pitch = 0;
integral_limit_pitch = 5;

for i = 2:length(t)
    airspeed_error = V_target - V(i-1);
    error_sum_speed = max(min(error_sum_speed + airspeed_error * dt, integral_limit_speed), -integral_limit_speed);
    error_derivative_speed = (airspeed_error - previous_error_speed) / dt;
    throttle(i) = max(min(throttle(i-1) + Kp_speed * airspeed_error + Ki_speed * error_sum_speed + Kd_speed * error_derivative_speed, max_throttle), min_throttle);
    previous_error_speed = airspeed_error;
    V(i) = V(i-1) + (throttle(i) - 0.5) * dt * 100;

    altitude_error = target_altitude - Z(i-1);
    pitch_error = deg2rad(-3) + (V_target - V(i)) * 0.01 + 0.0001 * altitude_error - pitch_angle(i-1);
    error_sum_pitch = max(min(error_sum_pitch + pitch_error * dt, integral_limit_pitch), -integral_limit_pitch);
    error_derivative_pitch = (pitch_error - previous_error_pitch) / dt;
    pitch_correction = Kp_pitch * pitch_error + Ki_pitch * error_sum_pitch + Kd_pitch * error_derivative_pitch;
    pitch_angle(i) = pitch_angle(i-1) + pitch_correction * dt;
    previous_error_pitch = pitch_error;

    Vz = V(i) * sin(pitch_angle(i));
    X(i) = X(i-1) + V(i) * dt * cos(roll_angle(i-1));
    Y(i) = Y(i-1) + V(i) * dt * sin(roll_angle(i-1));
    Z(i) = Z(i-1) + Vz * dt;

    if Z(i) > bank_threshold_alt
        roll_target(i) = bank_angle_max;
    elseif Z(i) > leveling_threshold
        roll_target(i) = bank_angle_max * (Z(i) - leveling_threshold) / (bank_threshold_alt - leveling_threshold);
    else
        roll_target(i) = zero_roll_angle;
    end

    roll_error = roll_target(i) - roll_angle(i-1);
    error_sum_roll = max(min(error_sum_roll + roll_error * dt, integral_limit_roll), -integral_limit_roll);
    error_derivative_roll = (roll_error - previous_error_roll) / dt;
    roll_angle(i) = roll_angle(i-1) + (Kp_roll * roll_error + Ki_roll * error_sum_roll + Kd_roll * error_derivative_roll) * dt;
    previous_error_roll = roll_error;
end

figure;
plot(t, roll_target, 'b', t, roll_angle, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Roll Angle (rad)');
title('PID Control of Roll Angle');
grid on;
legend('Target Roll', 'Actual Roll');

figure;
plot(t, pitch_angle, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Pitch Angle (rad)');
title('PID Control of Pitch Angle');
grid on;
legend('Pitch Angle');

% 3D Aircraft Simulation
figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
title('3D Aircraft Descent Simulation');
view(3);
xlim([0 max(X)*1.05]);
ylim([-10000, 10000]);
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
    T_translate = makehgtform('translate', [X(i) Y(i) Z(i)]);
    T_rotate_roll = makehgtform('xrotate', roll_angle(i));
    T_rotate_pitch = makehgtform('yrotate', -pitch_angle(i));
    
    T = T_translate * T_offset_inv * T_rotate_pitch * T_rotate_roll * T_offset;
    set(hgt, 'Matrix', T);
    set(trail, 'XData', X(1:i), 'YData', Y(1:i), 'ZData', Z(1:i));
    pause(0.005);
end
