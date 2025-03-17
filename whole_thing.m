clear; clc; close all;

%% Simulation parameters
dt = 0.02;
T_total = 1000;
t = 0:dt:T_total;

stage_level = 1;

%% Aircraft and takeoff parameters
runwayLength = 3048;
acceleration = 1.08;
t_rotation_start = sqrt(2 * runwayLength / acceleration);
V_takeoff = acceleration * t_rotation_start;
rotation_duration = 20;
initial_pitch = deg2rad(15);
climb_rate = (15);
tau_bank = 5;  % Time constant for smoothing bank angle change

ascend_velocity = 122; %commecial airplanes typically ascend at 250-300mph
V_ascend = V_takeoff;
final_velocity = 274.4;
V_cruise = V_ascend;
cruise_accel = 5;
target_altitude = 10000;

%% HEADING PID
psi_desired_initial = deg2rad(0);  % Initial desired heading
psi_desired_final = deg2rad(28);    % Final desired heading for ramp-up
ramp_duration = 50;                  % Duration to ramp up the heading
Kp = 15; Ki = 0.35; Kd = 20;
int_err = 0; err_prev = 0;
max_bank_angle = deg2rad(25);
turn_rate_constant = 9.81 / V_takeoff;

%% PITCH PID
Kp_pitch = 5; Ki_pitch = 0.1; Kd_pitch = 0.5;
int_err_pitch = 0; err_prev_pitch = 0;
pitch_tau = 5;

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

pitch_error_log = zeros(size(t));
P_pitch_log = zeros(size(t));
I_pitch_log = zeros(size(t));
D_pitch_log = zeros(size(t));

%% LQR controller
A=[-.0558 -.9968 .0802 .0415; .598 -.115 -.0318 0; -3.05 .388 -.4650 0; 0 0.0805 1 0];
B=[ .00729 0; -0.475 0.00775; 0.153 0.143; 0 0];
C=[0 1 0 0; 0 0 0 1];
D=[0 0;0 0];
sys = ss(A,B,C,D);


% % Extract LQR Response
% yaw_rate = Yaw(:,1); % Yaw response
% bank_angle = Yaw(:,end); % Roll response (bank angle)

%% Flight path loop
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
    elseif current_time <= 525
        if V_ascend < ascend_velocity
            V_ascend = V_ascend + acceleration * dt;
        end
        
        X(i) = X(i-1) + V_ascend * dt * cos(heading_angle(i-1));
        Y(i) = Y(i-1) + V_ascend * dt * sin(heading_angle(i-1));
        
        % Desired pitch control to maintain climb rate
        desired_pitch = asin(climb_rate / V_takeoff);
        pitch_error = desired_pitch - pitch_angle(i-1);
        int_err_pitch = int_err_pitch + pitch_error * dt;
        derr_pitch = (pitch_error - err_prev_pitch) / dt;
        err_prev_pitch = pitch_error;
        
        P_pitch = Kp_pitch * pitch_error;
        I_pitch = Ki_pitch * int_err_pitch;
        D_pitch = Kd_pitch * derr_pitch;
        
        pitch_command = P_pitch + I_pitch + D_pitch;
        pitch_angle(i) = pitch_angle(i-1) + (pitch_command - pitch_angle(i-1)) * (dt / pitch_tau);
        
%         pitch_angle(i) = pitch_angle(i-1) + (P_pitch + I_pitch + D_pitch) * dt;
        Z(i) = Z(i-1) + sin(pitch_angle(i)) * V_ascend * dt;

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

        pitch_error_log(i) = pitch_error;
        P_pitch_log(i) = P_pitch;
        I_pitch_log(i) = I_pitch;
        D_pitch_log(i) = D_pitch;
    %accelerate to cruise velocity and level out plane
    elseif stage_level
        if V_cruise < final_velocity
            V_cruise = V_cruise + cruise_accel * dt;
        end
                 
        X(i) = X(i-1) + V_cruise * dt * cos(heading_angle(i-1));
        Y(i) = Y(i-1) + V_cruise * dt * sin(heading_angle(i-1));
        
%         % pitch control
%         desired_descent_rate = (Z(i)-target_altitude) / (((final_velocity - ascend_velocity)/cruise_accel)-current_time);
% %         desired_pitch = asin(desired_descent_rate / V_cruise);
%         desired_pitch = 0;
        % Smooth transition from climb to level flight
        transition_duration = 10;
        if current_time > 525 && current_time <= 525 + transition_duration
            desired_pitch = initial_pitch * (1 - (current_time - 525) / transition_duration);
        else
            desired_pitch = 0;
        end

        pitch_error = desired_pitch - pitch_angle(i-1);

        int_err_pitch = int_err_pitch + pitch_error * dt;
        derr_pitch = (pitch_error - err_prev_pitch) / dt;
        err_prev_pitch = pitch_error;

        P_pitch = Kp_pitch * pitch_error;
        I_pitch = Ki_pitch * int_err_pitch;
        D_pitch = Kd_pitch * derr_pitch;

        pitch_command = P_pitch + I_pitch + D_pitch;

        % Limit pitch rate change for realism
        pitch_angle(i) = pitch_angle(i-1) + (pitch_command - pitch_angle(i-1)) * (dt / pitch_tau);

        % Update altitude based on pitch
        Z(i) = Z(i-1) + V_cruise * dt * sin(pitch_angle(i));
        
        if Z(i) > target_altitude
            stage_level = 0;
            stage_cruise = 1;
        end

%         if Z(i) <= target_altitude
%             Z(i) = target_altitude;
%             desired_descent_rate = 0;
%         end

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

        pitch_error_log(i) = pitch_error;
        P_pitch_log(i) = P_pitch;
        I_pitch_log(i) = I_pitch;
        D_pitch_log(i) = D_pitch;
    elseif stage_cruise
        pitch_error = desired_pitch - pitch_angle(i-1);

        int_err_pitch = int_err_pitch + pitch_error * dt;
        derr_pitch = (pitch_error - err_prev_pitch) / dt;
        err_prev_pitch = pitch_error;

        P_pitch = Kp_pitch * pitch_error;
        I_pitch = Ki_pitch * int_err_pitch;
        D_pitch = Kd_pitch * derr_pitch;

        pitch_command = P_pitch + I_pitch + D_pitch;

        % Limit pitch rate change for realism
        pitch_angle(i) = pitch_angle(i-1) + (pitch_command - pitch_angle(i-1)) * (dt / pitch_tau);

        % Update altitude based on pitch
        Z(i) = Z(i-1) + V_cruise * dt * sin(pitch_angle(i));
        
        X(i) = X(i-1) + V_cruise * dt * cos(heading_angle(i-1));
        Y(i) = Y(i-1) + V_cruise * dt * sin(heading_angle(i-1));
        
        
        
        pitch_error_log(i) = pitch_error;
        P_pitch_log(i) = P_pitch;
        I_pitch_log(i) = I_pitch;
        D_pitch_log(i) = D_pitch;
        
    end
    
end

% Plot pitch PID data
figure;
subplot(2, 1, 1);
plot(t, pitch_error_log, 'k');
xlabel('Time (s)'); ylabel('Pitch Error (rad)');
title('PID Pitch Error');
grid on;

subplot(2, 1, 2);
plot(t, P_pitch_log, 'r', t, I_pitch_log, 'g', t, D_pitch_log, 'b');
xlabel('Time (s)');
ylabel('PID Terms');
title('PID Pitch Contributions');
legend('P', 'I', 'D');
grid on;


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
scaleFactor = 5;
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


figure; hold on; grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
title('3D Aircraft Flight Path');
plot3(X, Y, Z, 'b-', 'LineWidth', 1.5);

scatter3(X(1), Y(1), Z(1), 50, 'go', 'filled'); % Start point
scatter3(X(end), Y(end), Z(end), 50, 'ro', 'filled'); % End point

% Set axis limits for better visualization
xlim([min(X) - 500, max(X) + 500]);
ylim([min(Y) - 500, max(Y) + 500]);
zlim([0, max(Z) + 500]);

legend('Flight Path', 'Start', 'End');
view(3); % 3D view


% % Animation loop
% for i = 1:length(t)
%     T_translate = makehgtform('translate', [X(i), Y(i), Z(i) + ground_clearance]);
%     T_yaw = makehgtform('zrotate', heading_angle(i));
%     T_pitch = makehgtform('yrotate', -pitch_angle(i));
%     T_roll = makehgtform('xrotate', -bank_angle(i));
% 
%     % Correct transformation order: translate → yaw → pitch → roll
%     T = T_translate * T_yaw * T_pitch * T_roll;
%     
%     set(hgt, 'Matrix', T);
%     set(trail, 'XData', X(1:i), 'YData', Y(1:i), 'ZData', Z(1:i) + ground_clearance);
%     pause(0.005);
% end


