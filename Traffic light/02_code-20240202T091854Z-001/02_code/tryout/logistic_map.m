% Define parameter range and resolution
r_min = 0;
r_max = 4;
tic
num_points = 50000;
r_vals = linspace(r_min, r_max, num_points);

% Initial condition
x0 = 0.2;

% Number of iterations to discard transient behavior
transient_steps = 200;

% Maximum number of iterations for convergence
max_iterations = 1000;

% Store final values for each parameter
final_values = zeros(num_points, 1);

% Loop through parameter values
for i = 1:num_points
  % Iterate the logistic map
  x = x0;
  for j = 1:max_iterations
    x = r_vals(i) * x * (1 - x);
    % Discard transient behavior
    if (j > transient_steps)
      final_values(i) = x;
      break;
    end
  end
end

% Plot the bifurcation diagram
plot(r_vals, final_values, 'k.');
xlabel('Parameter (r)');
ylabel('Final Value (x)');
title('Bifurcation Diagram of Logistic Map');

% Optional: Color points based on convergence speed
colors = jet(max_iterations);
scatter(r_vals, final_values, 10, "blue");
colorbar;
toc
