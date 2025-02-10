clear; clc; close all;

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

D = [0 0;
     0 0];

sys = ss(A,B,C,D);
tspan = [0 20]; 
x0 = [0.1; 0; 0; 0]; 
u = [0; 0];  
dxdt = @(t, x) A*x + B*u; 
[t_out, x_out] = ode45(dxdt, tspan, x0);
y_out = (C * x_out')';

[V,E]=eig(A);
disp(V)
disp(E)

figure;
subplot(4,1,1);
plot(t_out, x_out(:,1), 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel({'Sideslip Angle'; '(β) [rad]'},'FontSize',10);
title('Open Loop Response at Mach 0.8, 40,000 ft');
grid on;
legend('Sideslip Angle (\beta)');

subplot(4,1,2);
plot(t_out, y_out(:,1), 'b', 'LineWidth', 1.5);  
xlabel('Time (s)'); ylabel({'Yaw Rate (r)'; '[rad/s]'},'FontSize',10);
grid on;
legend('Yaw Rate (r)');

subplot(4,1,3);
plot(t_out, y_out(:,2), 'r', 'LineWidth', 1.5);  
xlabel('Time (s)'); ylabel({'Bank Angle'; '(\phi) [rad]'},'FontSize',10);
grid on;
legend('Bank Angle (\phi)');

subplot(4,1,4);
plot(t_out, x_out(:,4), 'm', 'LineWidth', 1.5);  
xlabel('Time (s)'); ylabel({'Pitch Rate'; '(q) [rad/s]'},'FontSize',10);
grid on;
legend('Pitch Rate (q)');

sgtitle('Boeing 747 Open-Loop Response');

figure;
rlocus(sys(1, 1));  
grid on;
title('Root Locus');
xlabel('Real Axis');
ylabel('Imaginary Axis');
