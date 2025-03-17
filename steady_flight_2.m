clear; clc; close all;

%% Define the plant
A = [-0.0558  -0.9968  0.0802  0.0415;
      0.5980  -0.1150 -0.0318  0;
     -3.0500   0.3880 -0.4650  0;
      0        0.0805  1       0];

B = [ 0.00729   0;
     -0.47500   0.00775;
      0.15300   0.14300;
      0         0];

C = [0 1 0 0;  
     0 0 0 1]; 

D = zeros(2,2);

% Create the plant as a state-space system
sys = ss(A, B, C, D);

%% Design a State-Space PID Controller
Kp = [0 0.001 0 0; 
      0 0 0 0.001];  % Proportional gain (2x4)
Ki = [0.5 0; 
      0 0.5];      % Integral gain (2x2)

% Augment the system with integral states
A_aug = [A, zeros(4,2); -C, zeros(2,2)]; % (6×6)
B_aug = [B; zeros(2,2)];  % (6×2)

% Construct K_aug correctly
K_aug = [Kp, Ki]; % Now (2×6)

% Closed-loop state-space system
A_cl = A_aug - B_aug * K_aug; % (6×6) - (6×2) * (2×6) = (6×6)
B_cl = zeros(size(A_cl, 1), 2); % Control inputs are zero (6×2)
C_cl = [C, zeros(2,2)]; % Output still selects relevant states (2×6)
D_cl = D; % (2×2)

CL_sys = ss(A_cl, B_cl, C_cl, D_cl);

%% Simulation parameters
t = linspace(0, 20, 2000);  % simulate for 20 seconds
x0 = [0.1; 0; 0; 0; 0; 0];  % Initial conditions (including integral states)

% Define the reference input for each output (zero for now)
r = zeros(length(t), 2);

%% Simulate the closed-loop response
[y, t_out, x_cl] = lsim(CL_sys, r, t, x0);

%% Plot the outputs
figure;
subplot(2,1,1);
plot(t_out, y(:,1), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Output 1');
title('Closed-Loop Response (Output 1)');
grid on;

subplot(2,1,2);
plot(t_out, y(:,2), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Output 2');
title('Closed-Loop Response (Output 2)');
grid on;
