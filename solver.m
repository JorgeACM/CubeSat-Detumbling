% Script: solver(t)
% 2023_24

% Setup the global variables
global v1 v2 v3

% Set the simulation duration
t_max = 100;

% Simulate the attitude of the spacecraft for t seconds duration
[t y] = ode45(@attitude_model, [0 t_max], [0 0 0 1 1 2 3 0 0 0]);

% Plot the results
figure(1);
plot(t, y(:,5));
xlabel('time');
ylabel('x-axis rotation');

figure(2);
plot(t, y(:,6));
xlabel('time');
ylabel('y-axis rotation');

figure(3);
plot(t, y(:,7));
xlabel('time');
ylabel('z-axis rotation');
