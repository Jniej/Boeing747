clear; clc; close all;

% Simulation parameters
dt = 0.02;
T_total = 250;
t = 0:dt:T_total;

% Aircraft and descent parameters
initial_altitude = 10000;
target_altitude = 1000;
V_aircraft = 250;    % Constant airspeed in m/s
psi_desired = 0; % Fixed heading of 0 degrees (North)
psi_desired_final = 0;
ramp_duration = 50;

% Target descent rate based on velocity and pitch
desired_descent_rate = (target_altitude - initial_altitude) / T_total; % m/s (negative for descent)

% Heading PID parameters
Kp_heading = 15; Ki_heading = 0.35; Kd_heading = 20;
int_err_heading = 0; err_prev_heading = 0;
max_bank_angle = deg2rad(25);
turn_rate_constant = 9.81 / V_aircraft;

% Pitch PID parameters
Kp_pitch = 0.8; Ki_pitch = 0.02; Kd_pitch = 0.5;
int_err_pitch = 0; err_prev_pitch = 0;

% State variables
X = zeros(size(t));
Y = zeros(size(t));
Z = initial_altitude * ones(size(t));
pitch_angle = zeros(size(t));
heading_angle = zeros(size(t));
bank_angle = zeros(size(t));

heading_error_log = zeros(size(t));
pitch_error_log = zeros(size(t));

% Flight path loop
for i = 2:length(t)
    current_time = t(i);
    start_descent = 15;
     if current_time < 5
         desired_descent_rate = 0;
     elseif current_time < start_descent
         desired_descent_rate = 100;
     else
         desired_descent_rate = (target_altitude - Z(i)) / (T_total - start_descent);
         psi_desired_final = deg2rad(28);
    end
        
    % Update position based on heading and velocity
    X(i) = X(i-1) + V_aircraft * dt * cos(heading_angle(i-1));
    Y(i) = Y(i-1) + V_aircraft * dt * sin(heading_angle(i-1));

    %% === PITCH CONTROL ===
    % Compute target pitch based on descent rate
    desired_pitch = asin(desired_descent_rate / V_aircraft);
    pitch_error = desired_pitch - pitch_angle(i-1);
    
    int_err_pitch = int_err_pitch + pitch_error * dt;
    derr_pitch = (pitch_error - err_prev_pitch) / dt;
    err_prev_pitch = pitch_error;
    
    P_pitch = Kp_pitch * pitch_error;
    I_pitch = Ki_pitch * int_err_pitch;
    D_pitch = Kd_pitch * derr_pitch;

    pitch_command = P_pitch + I_pitch + D_pitch;
    pitch_angle(i) = pitch_angle(i-1) + pitch_command * dt;

    % Update altitude based on pitch
    Z(i) = Z(i-1) + V_aircraft * dt * sin(pitch_angle(i));
    
    % Stop descent at target altitude
    if Z(i) <= target_altitude
        Z(i) = target_altitude;
        desired_descent_rate = 0;  % Level off at target altitude
    end
    
    %% === HEADING CONTROL ===
    if (start_descent < current_time) && (current_time <= ramp_duration + start_descent)
        psi_desired = psi_desired + (psi_desired_final - psi_desired) * (current_time / ramp_duration);
    else
        psi_desired = psi_desired_final;  % Once the ramp is complete, set the final desired heading
    end
    heading_error = psi_desired - heading_angle(i-1);
    
    int_err_heading = int_err_heading + heading_error * dt;
    derr_heading = (heading_error - err_prev_heading) / dt;
    err_prev_heading = heading_error;
    
    P_heading = Kp_heading * heading_error;
    I_heading = Ki_heading * int_err_heading;
    D_heading = Kd_heading * derr_heading;
    
    bank_command = P_heading + I_heading + D_heading;
    bank_command = min(max(bank_command, -max_bank_angle), max_bank_angle);
    
    % Smooth bank angle using a first-order lag
    tau_bank = 5;
    bank_angle(i) = bank_angle(i-1) + (bank_command - bank_angle(i-1)) * (dt / tau_bank);
    
    % Update heading based on bank angle
    turn_rate = turn_rate_constant * tan(bank_angle(i));
    heading_angle(i) = heading_angle(i-1) + turn_rate * dt;

    % Log data
    heading_error_log(i) = heading_error;
    pitch_error_log(i) = pitch_error;
end

%% === PLOTS ===
figure;
subplot(2, 1, 1);
plot(t, pitch_error_log, 'k');
xlabel('Time (s)'); ylabel('Pitch Error (rad)');
title('PID Pitch Error');
grid on;

subplot(2, 1, 2);
plot(t, heading_error_log, 'k');
xlabel('Time (s)'); ylabel('Heading Error (rad)');
title('PID Heading Error');
grid on;

%% === VISUALIZATION ===
figure; hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
title('3D Aircraft Descent Simulation');
view(3);
% view([0 -1 0]);

zlim([0, initial_altitude * 1.2]);
xlim([min(X) - 500, max(X) + 500]);
ylim([min(Y) - 500, max(Y) + 500]);

% Load 3D model
aircraftModel = stlread('747.stl');
scaleFactor = 1;
vertices = aircraftModel.Points * scaleFactor;
Rx = makehgtform('zrotate', -pi/2);
rotatedPoints = (Rx(1:3,1:3) * vertices')';
aircraft = patch('Faces', aircraftModel.ConnectivityList, 'Vertices', rotatedPoints, 'FaceColor', 'blue', 'EdgeColor', 'none');

% Setup animation transformation
hgt = hgtransform; set(aircraft, 'Parent', hgt);
trail = plot3(nan, nan, nan, 'r-', 'LineWidth', 1.5);

% Animation loop
for i = 1:length(t)
    T_translate = makehgtform('translate', [X(i), Y(i), Z(i)]);
    T_yaw = makehgtform('zrotate', heading_angle(i));
    T_pitch = makehgtform('yrotate', -pitch_angle(i));
    T_roll = makehgtform('xrotate', -bank_angle(i));

    % Correct transformation order: translate → yaw → pitch → roll
    T = T_translate * T_yaw * T_pitch * T_roll;
    
    set(hgt, 'Matrix', T);
    set(trail, 'XData', X(1:i), 'YData', Y(1:i), 'ZData', Z(1:i));
    pause(0.005);
end
