%% Calculation of the end-effector position
clear, clc

% Joint angle configuration (in radians)
th1 = 2 * pi;  % Angle of the first joint
th2 = 2 * pi;  % Angle of the second joint
th3 = 2 * pi;  % Angle of the third joint

% Display the angles used
disp(['Theta1: ', num2str(th1), ' rad']);
disp(['Theta2: ', num2str(th2), ' rad']);
disp(['Theta3: ', num2str(th3), ' rad']);

% Modified DH parameters
alpha = [0, -pi/2, 0];   
d = [0, 25, 20];         
theta = [th1, th2, th3]; 
r = [45, 20, 15];        

% Define the DH table for the RRR robot
DH_params = [
    0,        0,        th1,     45;  % First joint
   -pi/2,     25,       th2,     20;  % Second joint
    0,        20,       th3,     15   % Third joint
];

disp('DH Table:');
disp(array2table(DH_params, 'VariableNames', {'alpha', 'd', 'theta', 'r'}));

% Calculate the transformation matrix using Denavit-Hartenberg convention
[T0Tn, entities] = DenaHart(alpha, d, theta, r);

% Calculate the final position of the end-effector
pos_effector = T0Tn(1:3, 4); % Extract X, Y, Z coordinates
disp('End-effector position (X, Y, Z):');
disp(pos_effector);

% Display the final transformation matrix of the end-effector
disp('Final transformation matrix of the end-effector with modified DH:');
disp(T0Tn);

% 3D visualization of the robot in its home position
f1 = figure('Name', 'RRR Home Position', 'NumberTitle', 'off');
set(f1, 'WindowState', 'maximized');
RRR3D(th1, th2, th3);

%%
% Define a unit vector along the Z-axis
K = [0; 0; 1];

% Access transformation matrices from the struct 'entities'
T0T1 = entities(1).ele; % First transformation matrix 
T0T2 = entities(2).ele; % Second transformation matrix 
T0T3 = entities(3).ele; % Third transformation matrix 

% Extract rotation matrices (Z axes) for each joint
Z0Z0 = eye(3) * K;               
Z0Z1 = T0T1(1:3, 1:3) * K;       
Z0Z2 = T0T2(1:3, 1:3) * K;       

% Extract position vectors for each joint
P0P0 = [0; 0; 0];                
P0P1 = T0T1(1:3, 4);             
P0P3 = T0T3(1:3, 4);             

% Revolute joint calculations (cross product for linear velocity, Z for angular velocity)
J1_revolute = [cross(Z0Z0, (P0P3 - P0P0)); Z0Z0]; % For joint 1
J2_revolute = [cross(Z0Z1, (P0P3 - P0P1)); Z0Z1]; % For joint 2
J3_revolute = [cross(Z0Z2, (P0P3 - P0P1)); Z0Z2]; % For joint 3

% Construct the full Jacobian matrix J
J = [J1_revolute, J2_revolute, J3_revolute];

% Display the Jacobian matrix
disp('Jacobian matrix J:');
disp(J);

%%

% Simulation parameters
tf = 15;         % Simulation time in seconds
freq = 3;        % Controller frequency in Hz

% Create a time vector based on the frequency
t = 0:1/freq:tf;  % From 0 to tf with a step size of 1/freq (3 Hz)

% Pre-allocate arrays for joint angles and other parameters
th1 = zeros(1, length(t));
th2 = zeros(1, length(t));
th3 = zeros(1, length(t));

th1_i = 0;         % Initial th1
th1_f = pi/4;      % Final th1

th2_i = 0;         % Initial th2
th2_f = pi/4;      % Final th2

th3_i = 0;         % Initial th3
th3_f = pi/4;      % Final th3

% Create a figure
figure;

% Simulation loop
for i = 1:length(t)
    
    % Compute the cubic interpolation for each joint
    r = 10*(t(i)/tf)^3 - 15*(t(i)/tf)^4 + 6*(t(i)/tf)^5;  % Cubic trajectory
    
    % Revolute joints
    th1(i) = th1_i + (th1_f - th1_i) * r;  
    th2(i) = th2_i + (th2_f - th2_i) * r;  
    th3(i) = th3_i + (th3_f - th3_i) * r;  

    % Modified DH parameters
    alpha_i = [0, -pi/2, 0];            % Twist angles
    d_i = [0, 25, 20];                  % Z displacements
    theta_i = [th1_i, th2_i, th3_i];    % Rotation angles
    r_i = [45, 20, 15];                 % X displacements

    % Obtain end-effector coordinates (X, Y, Z)
    [T0Tn_i, entities_i] = DenaHart(alpha_i, d_i, theta_i, r_i);
    D = T0Tn_i(1:3, 4);
    
    % Call the 3D visualization function for the RRR robot
    % Subplots
    subplot(1,2,1);
    RRR3D(th1(i), th2(i), th3(i)); 
    title('3D View');
    drawnow;
    
    subplot(1,2,2);
    RRR3D(th1(i), th2(i), th3(i)); 
    view(2); % Top view
    title('Top View');
    drawnow;

end

%%
% Positions, Velocities & Accelerations
for i = 1:length(t)
    r = 10*(t(i)/tf)^3 - 15*(t(i)/tf)^4 + 6*(t(i)/tf)^5;                % Cubic trajectory r(t)
    r_dot = (30*(t(i)^2/tf^3) - 60*(t(i)^3/tf^4) + 30*(t(i)^4/tf^5));   % First derivative of r(t)
    r_ddot = (60*(t(i)/tf^3) - 180*(t(i)^2/tf^4) + 120*(t(i)^3/tf^5));  % Second derivative of r(t)
    
    th1(i) = th1_i + (th1_f - th1_i)*r;
    th1_dot(i) = (th1_f - th1_i)*r_dot;
    th1_ddot(i) = (th1_f - th1_i)*r_ddot;
    
    th2(i) = th2_i + (th2_f - th2_i)*r;
    th2_dot(i) = (th2_f - th2_i)*r_dot;
    th2_ddot(i) = (th2_f - th2_i)*r_ddot;
    
    th3(i) = th3_i + (th3_f - th3_i)*r;
    th3_dot(i) = (th3_f - th3_i)*r_dot;
    th3_ddot(i) = (th3_f - th3_i)*r_ddot;
    
end

% Plot Joints
f3 = figure('Name', 'Joint Visualization', 'NumberTitle', 'off');
set(f3, 'WindowState', 'maximized');

% Positions rad or m
ax1 = subplot(3,1,1); hold on;
h1 = plot(t, th1, 'DisplayName', 'th1');
h2 = plot(t, th2, 'DisplayName', 'th2');
h3 = plot(t, th3, 'DisplayName', 'th3');
title('Joint Positions');
legend('show');
xlabel('Time (s)');
ylabel('Position (rad or m)');

% Velocities rad/s or m/s
ax2 = subplot(3,1,2); hold on;
h1_dot = plot(t, th1_dot, 'DisplayName', 'th1\_dot');
h2_dot = plot(t, th2_dot, 'DisplayName', 'th2\_dot');
h3_dot = plot(t, th3_dot, 'DisplayName', 'th3\_dot');
title('Joint Velocities');
legend('show');
xlabel('Time (s)');
ylabel('Velocity (rad/s or m/s)');

% Accelerations rad/s^2 or m/s^2
ax3 = subplot(3,1,3); hold on;
h1_ddot = plot(t, th1_ddot, 'DisplayName', 'th1\_ddot');
h2_ddot = plot(t, th2_ddot, 'DisplayName', 'th2\_ddot');
h3_ddot = plot(t, th3_ddot, 'DisplayName', 'th3\_ddot');
title('Joint Accelerations');
legend('show');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2 or m/s^2)');



