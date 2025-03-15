clear; clc; close all;

% Simulation parameters
dt = 0.02;
T_total = 250;
t = 0:dt:T_total;

% Aircraft and takeoff parameters
runwayLength = 3048;
acceleration = 1.08;
t_rotation_start = sqrt(2 * runwayLength / acceleration);
V_takeoff = acceleration * t_rotation_start;
rotation_duration = 20;
initial_pitch = deg2rad(15);
climb_rate = 5;
tau_bank = 5;  % Time constant for smoothing bank angle change

% Turn and bank parameters
psi_desired_initial = deg2rad(0);  % Initial desired heading
psi_desired_final = deg2rad(28);    % Final desired heading for ramp-up
ramp_duration = 50;                  % Duration to ramp up the heading
Kp = 15; Ki = 0.35; Kd = 20;
int_err = 0; err_prev = 0;
max_bank_angle = deg2rad(25);
turn_rate_constant = 9.81 / V_takeoff;

% State variables
X = zeros(size(t));
Y = zeros(size(t));
Z = zeros(size(t));
pitch_angle = zeros(size(t));
heading_angle = zeros(size(t));
bank_angle = zeros(size(t));

heading_error_log = zeros(size(t));
P_term_log = zeros(size(t));
I_term_log = zeros(size(t));
D_term_log = zeros(size(t));


% Flight path loop
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
    
    % Climb
    else
        X(i) = X(i-1) + V_takeoff * dt * cos(heading_angle(i-1));
        Y(i) = Y(i-1) + V_takeoff * dt * sin(heading_angle(i-1));
        pitch_angle(i) = initial_pitch;
        Z(i) = Z(i-1) + sin(pitch_angle(i)) * V_takeoff * dt;

        % Gradual ramp of psi_desired over time
        if current_time <= ramp_duration
            psi_desired = psi_desired_initial + (psi_desired_final - psi_desired_initial) * (current_time / ramp_duration);
        else
            psi_desired = psi_desired_final;  % Once the ramp is complete, set the final desired heading
        end
        
        % Heading error for PID controller
        heading_error = psi_desired - heading_angle(i-1);
        int_err = int_err + heading_error * dt;
        derr = (heading_error - err_prev) / dt;
        err_prev = heading_error;
        
        P_term = Kp * heading_error;
        I_term = Ki * int_err;
        D_term = Kd * derr;

        bank_command = Kp * heading_error + Ki * int_err + Kd * derr;
        bank_command = min(max(bank_command, -max_bank_angle), max_bank_angle);

        % Smooth bank angle using a first-order lag
        bank_angle(i) = bank_angle(i-1) + (bank_command - bank_angle(i-1)) * (dt / tau_bank);

        % Update heading based on bank angle
        turn_rate = turn_rate_constant * tan(bank_angle(i));
        heading_angle(i) = heading_angle(i-1) + turn_rate * dt;
        
        % Log data
        heading_error_log(i) = heading_error;
        P_term_log(i) = P_term;
        I_term_log(i) = I_term;
        D_term_log(i) = D_term;

    end
end

figure;
subplot(3, 1, 1);
plot(t, heading_error_log, 'k');
xlabel('Time (s)'); ylabel('Heading Error (rad)');
title('PID Heading Error');
grid on;

subplot(3, 1, 2);
plot(t, bank_angle, 'b');
xlabel('Time (s)'); ylabel('Bank Angle (rad)');
title('Bank Angle');
grid on;

subplot(3, 1, 3);
plot(t, P_term_log, 'r', t, I_term_log, 'g', t, D_term_log, 'b');
xlabel('Time (s)');
ylabel('PID Terms');
ylim([-15 30]);
title('PID Contributions');
legend('P', 'I', 'D');
grid on;

% Visualization setup
figure; hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
title('3D Aircraft Takeoff Simulation with Banked Turn and Ramp Input');
view(3);

ground_clearance = 5; maxAltitude = max(Z) + ground_clearance;
if maxAltitude < 50, maxAltitude = 50; end
zlim([0, maxAltitude * 1.2]);

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
pivot_offset = [-20, 0, -10];
T_offset = makehgtform('translate', pivot_offset);
T_offset_inv = makehgtform('translate', -pivot_offset);


% Animation loop
for i = 1:length(t)
    T_translate = makehgtform('translate', [X(i), Y(i), Z(i) + ground_clearance]);
    T_yaw = makehgtform('zrotate', heading_angle(i));
    T_pitch = makehgtform('yrotate', -pitch_angle(i));
    T_roll = makehgtform('xrotate', -bank_angle(i));

    % Correct transformation order: translate → yaw → pitch → roll
    T = T_translate * T_yaw * T_pitch * T_roll;
    
    set(hgt, 'Matrix', T);
    set(trail, 'XData', X(1:i), 'YData', Y(1:i), 'ZData', Z(1:i) + ground_clearance);
    pause(0.005);
end


