clear; clc; close all;

fig = figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
title('3D Aircraft Flight Animation');
view(3);

aircraftModel = stlread('747.stl');  
Rx = makehgtform('zrotate', -pi/2); 
rotatedPoints = (Rx(1:3, 1:3) * aircraftModel.Points')';

aircraft = patch('Faces', aircraftModel.ConnectivityList, ...
                 'Vertices', rotatedPoints, ...
                 'FaceColor', 'blue', 'EdgeColor', 'none');

A = [-0.0558  -0.9968  0.0802  0.0415;
      0.5980  -0.1150 -0.0318  0;
     -3.0500   0.3880 -0.4650  0;
      0        0.0805  1       0];

B = [ 0.00729   0;
     -0.47500   0.00775;
      0.15300   0.14300;
      0         0];

x0 = [0.1; 0; 0; 0]; 
tspan = linspace(0, 20, 200);  
dt = tspan(2) - tspan(1);
V = 273;  

dxdt = @(t, x) A*x; 
[t_out, x_out] = ode45(dxdt, tspan, x0);

yaw_rate = x_out(:,2);
bank_angle = x_out(:,4);
psi = cumtrapz(t_out, yaw_rate);

X = cumtrapz(t_out, V * cos(psi));  
Y = cumtrapz(t_out, V * sin(psi));  
Z = -cumtrapz(t_out, V * sin(bank_angle)) + 12192;

xlim([min(X) max(X)]);
ylim([min(Y) max(Y)]);
zlim([min(Z) max(Z)]);

t = hgtransform;
set(aircraft, 'Parent', t);

for i = 1:length(t_out)
    T = makehgtform('translate', [X(i) Y(i) Z(i)], ...
                    'zrotate', psi(i), 'xrotate', bank_angle(i));
    
    set(t, 'Matrix', T);
    pause(0.05);
end
