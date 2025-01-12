%% Calculation of the end-effector position with trajectory
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

% Define the starting position of the profile
Px = pos_effector(1);
Py = pos_effector(2);
Pz = pos_effector(3);

% Generate the profile using the 'printing' function
A = printing(Px, Py, Pz);

% Plot the robot in its home position
f1 = figure('Name', 'RRR Home Position with Trajectory', 'NumberTitle', 'off');
set(f1, 'WindowState', 'maximized');
RRR3D(th1, th2, th3);

% Plot the trajectory of the geometry in 3D space
hold on;
plot3(A(:,1), A(:,2), A(:,3), 'r-', 'LineWidth', 2); % Trajectory in red
scatter3(A(:,1), A(:,2), A(:,3), 'b', 'filled'); % Points of the trajectory
title('RRR Robot with Trajectory of the Geometry');
legend('Robot Links', 'Trajectory Path', 'Trajectory Points');
hold off;
