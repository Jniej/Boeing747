function landing()
    % Clean simulation of aircraft landing from 200m altitude
    clear; clc; close all;
    
    %% Simulation Parameters
    dt = 0.05;                  % Time step (s)
    t_total = 140;              % Total simulation time (s)
    t = 0:dt:t_total;           % Time vector
    
    %% Aircraft Parameters (updated for realistic speeds)
    initial_altitude = 200;     % Starting altitude (m)
    mph_to_ms = 0.44704;       % Conversion factor
    approach_speed = 160 * mph_to_ms;  % ~150 mph (67 m/s) approach
    touchdown_speed = 130 * mph_to_ms; % ~130 mph (58 m/s) touchdown
    flare_altitude = 10;        % Altitude to begin flare (m)
    %touchdown_altitude = 8;     % Altitude considered as touchdown (m)
    runway_altitude = 10;        % Constant runway altitude
    
    %% Flight Control Parameters
    Kp_pitch = 1.2;
    Ki_pitch = 0.03;
    Kd_pitch = 0.7;
    
    Kp_speed = 0.15;
    Ki_speed = 0.005;
    Kd_speed = 0.08;
    
    %% Initialize State Variables
    n = length(t);
    initial_X = -4700;
    X = initial_X * ones(1,n);  % Position (m)
    Y = zeros(1,n);
    Z = initial_altitude*ones(1,n);
    pitch = zeros(1,n);         % Pitch angle (rad)
    speed = approach_speed*ones(1,n);
    
    % Control variables
    pitch_error_int = 0;
    prev_pitch_error = 0;
    speed_error_int = 0;
    prev_speed_error = 0;
    
    %% Main Simulation Loop
    touchdown_index = n; % Initialize to end of simulation in case no touchdown
    on_runway = false;
    
    for i = 2:n
        if on_runway
            % --- Post-touchdown deceleration ---
            decel = 1.21; % Realistic deceleration rate (m/s²)
            pitch_decay = deg2rad(2); % pitch reduction per second
            
            % Reduce speed to 0 gradually
            speed(i) = max(0, speed(i-1) - decel * dt);
            
            % Bring pitch down to 0 smoothly
            pitch(i) = max(0, pitch(i-1) - pitch_decay * dt);
            
            % Keep aircraft on runway at 8m altitude
            Z(i) = runway_altitude;
            X(i) = X(i-1) + speed(i)*cos(pitch(i))*dt;
            
            % If fully stopped, break the loop
            if speed(i) <= 0.1
                touchdown_index = i;
                break;
            end
            continue;
        end

        current_altitude = Z(i-1);
        current_speed = speed(i-1);
        
        % Determine desired parameters based on phase
        if current_altitude > flare_altitude
            % Approach phase
            desired_pitch = deg2rad(-3.0);
            desired_speed = approach_speed - (approach_speed - touchdown_speed) * ...
                          (1 - min(1, (current_altitude-flare_altitude)/(initial_altitude-flare_altitude)));
            max_sink_rate = 8;
            min_sink_rate = 5;
            desired_sink_rate = -min_sink_rate - (max_sink_rate-min_sink_rate)*...
                               (current_altitude-flare_altitude)/(initial_altitude-flare_altitude);
        else
            % Flare phase
            flare_progress = min(1, current_altitude/flare_altitude);
            desired_pitch = deg2rad(-3.0 + 8*(1-flare_progress));
            desired_speed = touchdown_speed + (approach_speed - touchdown_speed) * flare_progress * 0.3;
            desired_sink_rate = -max(0.2, 0.5 * flare_progress);
        end

        % Speed Control
        speed_error = desired_speed - current_speed;
        speed_error_int = speed_error_int + speed_error*dt;
        speed_error_deriv = (speed_error - prev_speed_error)/dt;

        speed_command = Kp_speed*speed_error + Ki_speed*speed_error_int + Kd_speed*speed_error_deriv;
        max_accel = 0.8;
        speed_change = sign(speed_command)*min(abs(speed_command), max_accel*dt);
        speed(i) = current_speed + speed_change;

        % Pitch Control
        if current_altitude < flare_altitude
            required_pitch = asin(desired_sink_rate / current_speed);
            desired_pitch = max(desired_pitch, required_pitch);
        end

        pitch_error = desired_pitch - pitch(i-1);
        pitch_error_int = pitch_error_int + pitch_error*dt;
        pitch_error_deriv = (pitch_error - prev_pitch_error)/dt;

        pitch_command = Kp_pitch*pitch_error + Ki_pitch*pitch_error_int + Kd_pitch*pitch_error_deriv;
        max_pitch_rate = deg2rad(4);
        pitch_change = sign(pitch_command)*min(abs(pitch_command), max_pitch_rate*dt);
        pitch(i) = pitch(i-1) + pitch_change;

        % Update Position
        X(i) = X(i-1) + speed(i)*cos(pitch(i))*dt;
        new_altitude = Z(i-1) + speed(i)*sin(pitch(i))*dt;
        
        % Prevent altitude from going below runway level
        if new_altitude < runway_altitude
            Z(i) = runway_altitude;
            on_runway = true;
            touchdown_index = i;
        else
            Z(i) = new_altitude;
        end

        % Check for Touchdown
        if (Z(i) <= runway_altitude + 0.1) || (current_altitude < flare_altitude && abs(desired_sink_rate) < 1)
            Z(i) = runway_altitude;
            on_runway = true;
            touchdown_index = i;
        end

        prev_pitch_error = pitch_error;
        prev_speed_error = speed_error;
    end
    
    %% Visualization
    figure('Position', [100 100 1200 800], 'Name', 'Landing Performance');
    
    % 3D Flight Path
    subplot(2,2,[1 3]);
    hold on; grid on; axis equal;
    plot3(X(1:touchdown_index), Y(1:touchdown_index), Z(1:touchdown_index), 'b-', 'LineWidth', 2);
    
    % Mark key points
    scatter3(0, 0, initial_altitude, 100, 'go', 'filled');
    scatter3(X(touchdown_index), Y(touchdown_index), Z(touchdown_index), 100, 'rx', 'LineWidth', 2);
    
    % Mark flare initiation if it occurred
    flare_index = find(Z <= flare_altitude, 1);
    if ~isempty(flare_index)
        scatter3(X(flare_index), Y(flare_index), Z(flare_index), 100, 'ms', 'filled');
    end
    
    % Runway at 8m altitude
    runway_start = 0;
    runway_length = 3000;
    patch([runway_start runway_start+runway_length runway_start+runway_length runway_start], ...
      [-50 -50 50 50], [runway_altitude runway_altitude runway_altitude runway_altitude], 'g', 'FaceAlpha', 0.2);
    
    xlabel('Distance (m)'); ylabel('Lateral (m)'); zlabel('Altitude (m)');
    title('3D Landing Trajectory');
    view(45, 30);
    
    legend_items = {'Flight Path', 'Start', 'Touchdown'};
    if ~isempty(flare_index)
        legend_items = [legend_items, {'Flare Initiation'}];
    end
    legend(legend_items, 'Location', 'northeast');
    
    % Altitude Profile
    subplot(2,2,2);
    plot(t(1:touchdown_index), Z(1:touchdown_index), 'LineWidth', 2);
    hold on;
    yline(flare_altitude, '--m', 'Flare Altitude');
    yline(runway_altitude, '--g', 'Runway Altitude');
    xlabel('Time (s)'); ylabel('Altitude (m)');
    title('Altitude vs Time');
    grid on;
    
    % Speed Profile
    subplot(2,2,4);
    plot(t(1:touchdown_index), speed(1:touchdown_index), 'LineWidth', 2);
    hold on;
    yline(speed(touchdown_index), '--r', 'Touchdown Speed');
    xlabel('Time (s)'); ylabel('Speed (m/s)');
    title('Speed vs Time');
    grid on;
    
    %% Improved Animation with 747 STL Model
    fig2 = figure('Position', [200 200 800 600], 'Name', 'Landing Animation');
    ax = axes(fig2);
    hold(ax, 'on'); grid(ax, 'on');
    axis(ax, 'equal');
    ylim(ax, [-100 100]);
    zlim(ax, [runway_altitude-5 initial_altitude+50]); % Adjusted for 8m runway
    view(ax, 45, 30);
    light('Position',[1 0 0],'Style','infinite');
    light('Position',[0 1 0],'Style','infinite');

    % Load and prepare 747 model
    try
        aircraftModel = stlread('747.stl');
        scaleFactor = 0.25;
        vertices = aircraftModel.Points * scaleFactor;
        
        % Initial rotation to align model correctly
        Rz = makehgtform('zrotate', -pi/2);
        rotatedPoints = (Rz(1:3,1:3) * vertices')';
        
        % Create patch object for the aircraft
        aircraft = patch('Faces', aircraftModel.ConnectivityList, 'Vertices', rotatedPoints, 'FaceColor', 'blue', 'EdgeColor', 'none');
        
        % Create transform object for the aircraft
        hgt = hgtransform('Parent', ax);
        set(aircraft, 'Parent', hgt);
        
        model_loaded = true;
    catch
        warning('747.stl not found. Using simple representation.');
        model_loaded = false;
        aircraft = plot3(ax, X(1), Y(1), Z(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    end

    % Create trail/path
    trail = plot3(ax, nan, nan, nan, 'b-', 'LineWidth', 1.5);

    % Runway patch at 8m altitude
    runway_patch = patch(ax, ...
        [0 runway_length runway_length 0], ...
        [-50 -50 50 50], ...
        [runway_altitude runway_altitude runway_altitude runway_altitude], 'g', 'FaceAlpha', 0.2);

    title(ax, 'Landing Animation');
    xlabel(ax, 'Distance (m)'); ylabel(ax, 'Lateral (m)'); zlabel(ax, 'Altitude (m)');

    % Animation loop
    frame_skip = 3;
    for k = 1:frame_skip:touchdown_index
        % Update position data
        x_pos = X(k);
        y_pos = Y(k);
        z_pos = Z(k);
        current_pitch = pitch(k);
        
        % Update trail
        trail.XData = [trail.XData x_pos];
        trail.YData = [trail.YData y_pos];
        trail.ZData = [trail.ZData z_pos];
        
        if model_loaded
            % Calculate transformation matrix
            T_trans = makehgtform('translate', [x_pos y_pos z_pos]);
            R_pitch = makehgtform('xrotate', current_pitch);
            set(hgt, 'Matrix', T_trans * R_pitch);
        else
            set(aircraft, 'XData', x_pos, 'YData', y_pos, 'ZData', z_pos);
        end
        
        % Move runway patch
        % set(runway_patch, 'XData', [0 runway_length runway_length 0] + x_pos);
        
        % Shift view
        xlim(ax, [x_pos-100 x_pos+300]);
        
        % Update status
        if Z(k) <= runway_altitude + 0.1
            status = 'ON RUNWAY';
        elseif Z(k) <= flare_altitude
            status = 'FLARE';
        else
            status = 'APPROACH';
        end
        
        title(ax, sprintf('Landing Animation - %s\nTime: %.1fs | Alt: %.1fm | Speed: %.1fm/s | Pitch: %.1f°', ...
                         status, t(k), Z(k), speed(k), rad2deg(pitch(k))));
        
        drawnow;
        pause(0.02);
    end

    % Final frame
    if speed(touchdown_index) <= 0.1
        title(ax, sprintf('LANDING COMPLETE\nFinal Position: %.1fm | Total Time: %.1fs', ...
                         X(touchdown_index), t(touchdown_index)));
        drawnow;
    end
end