% EE 4CL4 Project - 
% Estimating Mobile User Motion Using Angle-Only Measurements

% Clear workspace and figures
close all; clc;

% Parameters
v_user = 30; % User speed in m/s
v_sensor = 35; % Sensor speed in m/s
t = 0:1:29; % Time vector from 0 to 29 seconds
sigma = pi / 180; % Standard deviation in radians (1 degree)
max_iter = 10; % Number of iterations for Newton's method
num_realizations = 1000; % Number of noise realizations

% True user parameters
true_xi0 = 0;
true_eta0 = 0;
true_dotxi = v_user * cosd(45);
true_doteta = v_user * sind(45);

% User trajectory (moving Northeast)
xi_u = true_xi0 + true_dotxi * t;
eta_u = true_eta0 + true_doteta * t;

% Sensor trajectory
xi_p = zeros(size(t));
eta_p = zeros(size(t));

for i = 1:length(t)
    if t(i) <= 15
        % Sensor moving West
        xi_p(i) = 1000 - v_sensor * t(i);
        eta_p(i) = 0;
    else
        % Sensor moving North
        xi_p(i) = 1000 - v_sensor * 15;
        eta_p(i) = v_sensor * (t(i) - 15);
    end
end

%% Plotting Trajectories
figure;
plot(xi_u, eta_u, 'b-', 'LineWidth', 2);
hold on;
plot(xi_p, eta_p, 'r-', 'LineWidth', 2);
legend('User Trajectory', 'Sensor Trajectory', 'Location', 'Best');
xlabel('East (m)');
ylabel('North (m)');
title('Trajectories of User and Sensor');
grid on;
axis equal;
% Save the figure
saveas(gcf, 'trajectories.png');

%% Plotting Angle Measurements
% Compute true angles
theta = atan2(eta_u - eta_p, xi_u - xi_p);

% Add measurement noise
rng('default'); % For reproducibility
w = sigma * randn(size(theta));
z = theta + w;

% Convert angles to degrees for plotting
theta_deg = theta * 180 / pi;
z_deg = z * 180 / pi;

% Plotting the angles
figure;
plot(t, theta_deg, 'b-', 'LineWidth', 2);
hold on;
plot(t, z_deg, 'r.', 'MarkerSize', 10);
legend('True Angle', 'Measured Angle', 'Location', 'Best');
xlabel('Time (s)');
ylabel('Angle (degrees)');
title('True Angles and Measured Angles Over Time');
grid on;
% Save the figure
saveas(gcf, 'angles.png');

%% Newton's Method for Parameter Estimation
t_est = 1:1:30; % Time vector from 1 to 30 seconds

% Sensor positions at t_est
xi_p_est = zeros(size(t_est));
eta_p_est = zeros(size(t_est));
% Corrected time indexing for sensor positions
for i = 1:length(t_est)
    ti = t_est(i) - 1; % Adjusted to match time indexing from 0 to 29
    if ti <= 15
        xi_p_est(i) = 1000 - v_sensor * ti;
        eta_p_est(i) = 0;
    else
        xi_p_est(i) = 1000 - v_sensor * 15;
        eta_p_est(i) = v_sensor * (ti - 15);
    end
end

% True user positions at t_est
xi_u_est = true_xi0 + true_dotxi * t_est;
eta_u_est = true_eta0 + true_doteta * t_est;

% True angles at t_est
theta_est = atan2(eta_u_est - eta_p_est, xi_u_est - xi_p_est);

% Preallocate arrays for estimates
estimates = zeros(4, max_iter, num_realizations);

for realization = 1:num_realizations
    rng(realization); % For reproducibility
    % Add measurement noise
    w = sigma * randn(size(theta_est));
    z = theta_est + w;

    % Initial estimate
    x_est = [10; 10; 20; 20]; % [xi(0); eta(0); dot_xi; dot_eta]

    % Iterative estimation
    for iter = 1:max_iter
        % Compute estimated positions
        xi_est = x_est(1) + x_est(3) * t_est;
        eta_est = x_est(2) + x_est(4) * t_est;

        % Compute estimated angles
        theta_pred = atan2(eta_est - eta_p_est, xi_est - xi_p_est);

        % Compute residuals
        res = z - theta_pred;

        % Compute Jacobian matrix J
        J = zeros(length(t_est), 4);
        % Corrected Jacobian computation
        for i = 1:length(t_est)
            delta_x = xi_est(i) - xi_p_est(i);
            delta_y = eta_est(i) - eta_p_est(i);
            denom = delta_x^2 + delta_y^2;

            % Corrected partial derivatives
            dtheta_dxi0 = -delta_y / denom;
            dtheta_deta0 = delta_x / denom;
            dtheta_ddotxi = dtheta_dxi0 * (t_est(i) - 1);
            dtheta_ddoteta = dtheta_deta0 * (t_est(i) - 1);

            J(i, :) = [dtheta_dxi0, dtheta_deta0, dtheta_ddotxi, dtheta_ddoteta];
        end

        % Compute gradient and Hessian
        grad = -(1 / sigma^2) * J' * res';
        Hessian = (1 / sigma^2) * (J' * J);

        % Update estimate
        delta_x = Hessian \ grad;
        x_est = x_est - delta_x;

        % Store estimates
        estimates(:, iter, realization) = x_est;
    end
end

% Compute mean estimates over realizations
mean_estimates = mean(estimates, 3);

% Display final mean estimate
disp('Estimated Parameters after iterations (averaged over realizations):');
disp(['xi(0): ', num2str(mean_estimates(1, end))]);
disp(['eta(0): ', num2str(mean_estimates(2, end))]);
disp(['dot_xi: ', num2str(mean_estimates(3, end))]);
disp(['dot_eta: ', num2str(mean_estimates(4, end))]);

% Plot convergence of estimates
figure;
plot(1:max_iter, mean_estimates(1, :), '-o', 'LineWidth', 2);
hold on;
plot(1:max_iter, mean_estimates(2, :), '-o', 'LineWidth', 2);
plot(1:max_iter, mean_estimates(3, :), '-o', 'LineWidth', 2);
plot(1:max_iter, mean_estimates(4, :), '-o', 'LineWidth', 2);
legend('xi(0)', 'eta(0)', 'dot\_xi', 'dot\_eta', 'Location', 'Best');
xlabel('Iteration');
ylabel('Estimate Value');
title('Convergence of Parameter Estimates (Averaged Over Realizations)');
grid on;
% Save the figure
saveas(gcf, 'convergence.png');
