function [theta1, theta2] = IKP(Px, Py, L1, L2)
    %    % Solve inverse kinematics for the robot

    % Calculate the radial distance
    r = sqrt(Px^2 + Py^2);

    % Assign theta2 as given
    theta2 = -pi; % Fixed value as specified in the problem

    % Calculate gamma (angle to the target point)
    gamma = atan2(Py, Px);

    % Calculate alpha (angle using the sine rule)
    alpha = asin(L2 / r); 
    
    % Compute theta1 using the given formula
    theta1 = gamma - alpha;
end
