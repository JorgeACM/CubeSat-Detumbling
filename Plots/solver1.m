% Script: solver(t)
% 2023_24
clear
clc
% Setup the global variables
global v1 v2 v3 control ms md wd wp torque

% Set the simulation duration
% t_max = 100;
orbits = 3;
t_max = orbits*5833;
% t_max = 45;
n = sqrt(3.98e5/7000^3);
orbitPeriod = 2*pi()/n;

orbitPeriods= 1:1:orbits;

% Settling Time Parameters
n = sqrt(3.98e5/7000^3);
% yfinal = [0,0,0,1, 0, -n,0,0,0,0]';
yfinal = [0., -n, 0, n, -1, 1]';
yinit = [1, 2, 3, norm([1,2,3]), 1, 1]';
ms=990;
md=0.06;
% ms=994;
% md=0.06;
v3 = 146;

wd= 9;
wp = 5;
torque=[0, 0, 0, 0];

% control = "magnetorquer1";
% control = "magnetorquer2";
control = "wheels";

k=1;
fprintf("Starting ODE45...\n");
% Simulate the attitude of the spacecraft for t seconds duration
[t y] = ode45(@attitude_model, [0 t_max], [0 0 0 1 1 2 3 0 0 0]);
fprintf("ODE45 Finished\n");
% % Norm of Angular Velocities
    for j = 1:size(y,1)
        norms(j) = norm(y(j,5:7));
    end
    vel = cat(2, y(:,5:7), norms',y(:,4),y(:,4));
    % [settling_time,settling_period1] = calculate_settling_time(t, y(:,5));
    ST=[];
    % % Calculate Settling Time
    S = lsiminfo(vel(2:end,:),t(2:end), yfinal, yinit);
    % S = lsiminfo(vel(2:end,:),t(2:end), yfinal, yinit, 'SettlingTimeThreshold',0.05);
    % S = lsiminfo(vel,t, 'SettlingTimeThreshold',0.02);
    STx = S(1).SettlingTime;
    STy = S(2).SettlingTime;
    STz = S(3).SettlingTime;
    STnorm = S(4).SettlingTime;
    STq = S(5).SettlingTime;
    STq2 = S(6).SettlingTime;
    % STtotal= max([STx, STy, STz]);
    STtotal= max([STx, STy, STz, STq]);
    % ST = cat(1,ST,[md, ms, STx, STy, STz, STnorm, STq, STq2,STtotal]);
    % ST = cat(1,ST,[v3, STx, STy, STz, STnorm, STq, STq2, STtotal]);
    ST = cat(1,ST,[wd, wp, STx, STy, STz, STnorm, STq, STq2,STtotal]);

    

% Plot the results
figure(1);
plot(t, y(:,5));
hold on;
xlabel('time');
ylabel('rad/s');
xline(STx, 'r--');
xline(S(1).TransientTime, 'b--');
text(STx+0.5, 0.5, ['x = ',num2str(STx)], 'Color', 'red');
text(S(1).TransientTime+0.1, 0.8, ['x = ',num2str(S(1).TransientTime)], 'Color', 'blue');
legend('Wx', 'Settling Time', 'Transient Time');
title('X-axis rotation');
hold off;

figure(2);
plot(t, y(:,6));
hold on;
xlabel('time');
ylabel('rad/s');
xline(STy, 'r--');
xline(S(2).TransientTime, 'b--');
text(STy+0.5, 0.5, ['x = ',num2str(STy)], 'Color', 'red');
text(S(2).TransientTime+0.1, 0.8, ['x = ',num2str(S(2).TransientTime)], 'Color', 'blue');
legend('Wy', 'Settling Time', 'Transient Time');
title('Y-axis rotation');
hold off;

figure(3);
plot(t, y(:,7));
hold on;
xlabel('time');
ylabel('rad/s');
xline(STz, 'r--');
xline(S(3).TransientTime, 'b--');
text(STz+0.5, 0.5, ['x = ',num2str(STz)], 'Color', 'red');
text(S(3).TransientTime+0.1, 0.8, ['x = ',num2str(S(3).TransientTime)], 'Color', 'blue');
legend('Wz', 'Settling Time', 'Transient Time');
title('Z-axis rotation');
hold off;

settleTime=ST(end);
% % PLOT IN HOURS
% figure(4);
% plot(t/3600, y(:,5));
% hold on;
% plot(t/3600, y(:,6));
% plot(t/3600, y(:,7));
% xlabel('time [hours]','FontSize', 14);
% ylabel('Rotation Rates  [rad/s]','FontSize', 14);
% xline(settleTime/3600, 'b--', ['x = ', num2str(round(settleTime/3600,2))], 'FontSize', 11);
% for i=1:length(orbitPeriods)
%     xstr = num2str(orbitPeriods(i))+ " O P";
%     xline(orbitPeriods(i)*orbitPeriod/3600, 'k:', xstr);
% end
% % ylim([-.003, .08]);
% % xlim([3*orbitPeriod/3600,t_max/3600]);
% title('Rotation Rates in Body Frame','FontSize', 20);
% legend("X-axis", "Y-axis","Z-axis", "Settling Time","Orbital Periods",'FontSize', 12);
% % legend("\omega_{x}", "\omega_{y}","\omega_{z}", "Settling Time","Orbital Periods",'FontSize', 14);
% hold off

% PLOT IN SECONDS
figure(10);
subplot(2,1,2);
plot(t, y(:,5));
hold on;
plot(t, y(:,6));
plot(t, y(:,7));
xlabel('time [s]','FontSize', 14);
ylabel('Rotation Rates  [rad/s]','FontSize', 14);
xline(settleTime, 'b--', ['x = ', num2str(round(settleTime,2))], 'FontSize', 11);
for i=1:length(orbitPeriods)
    xstr = num2str(orbitPeriods(i))+ " O P";
    xline(orbitPeriods(i)*orbitPeriod, 'k:', xstr);
end
ylim([-.005, .005]);
xlim([0,45]);
txt = {'\rm\bullet', '\rm\uparrow','\omega_{y} ≈ -0.0011 rad/s', };
text(35, -.0018,txt,'FontSize', 12);
txt1 = {'\omega_{x} ≈ \omega_{z} ≈ 0 rad/s', '\rm\downarrow', '\rm\bullet',};
text(37.5, 8e-4,txt1,'FontSize', 12);
title('Zoom-in','FontSize', 18);
legend("X-axis", "Y-axis","Z-axis", "Settling Time","Orbital Periods",'FontSize', 12);
% legend("\omega_{x}", "\omega_{y}","\omega_{z}", "Settling Time","Orbital Periods",'FontSize', 14);
hold off

% PLOT IN SECONDS
% figure(6);
subplot(2,1,1);
plot(t, y(:,5));
hold on;
plot(t, y(:,6));
plot(t, y(:,7));
xlabel('time [s]','FontSize', 14);
ylabel('Rotation Rates  [rad/s]','FontSize', 14);
xline(settleTime, 'b--', ['x = ', num2str(round(settleTime,2))], 'FontSize', 11);
for i=1:length(orbitPeriods)
    xstr = num2str(orbitPeriods(i))+ " O P";
    xline(orbitPeriods(i)*orbitPeriod, 'k:', xstr);
end
xlim([0,45]);
ylim([-.3, 3]);
title({'Reaction Wheel Controller Rotation Rates in Body Frame Kd = 9, Kp = 5'},'FontSize', 20);
legend("X-axis", "Y-axis","Z-axis", "Settling Time","Orbital Periods",'FontSize', 12);
% legend("\omega_{x}", "\omega_{y}","\omega_{z}", "Settling Time","Orbital Periods",'FontSize', 14);
hold off

figure(5);
plot(t, y(:,1))
hold on
plot(t, y(:,2))
plot(t, y(:,3))
plot(t, y(:,4))

xlabel('time [s]');
ylabel('quaternion');
% yline(0, 'k');
xline(settleTime, 'b--', ['x = ', num2str(round(settleTime,2))]);
for i=1:length(orbitPeriods)
    xstr = num2str(orbitPeriods(i))+ " O P";
    xline(orbitPeriods(i)*orbitPeriod, 'k:', xstr);
end
legend("q1", "q2","q3","q4", "Orbital Periods",'fontsize', 12)
title('Quaternions','FontSize', 20);
hold off

% figure(6);
% plot(t, norms');
% hold on;
% xlabel('time');
% ylabel('rad/s');
% xline(STnorm, 'r--');
% xline(S(4).TransientTime, 'b--');
% text(STnorm+0.5, 0.5, ['x = ',num2str(STnorm)], 'Color', 'red');
% text(S(4).TransientTime+0.1, 0.8, ['x = ',num2str(S(4).TransientTime)], 'Color', 'blue');
% legend('Wnorm', 'Settling Time', 'Transient Time');
% title('Normalized rotation');
% hold off;

figure(7);
plot(t, y(:,8));
hold on
plot(t, y(:,9));
plot(t, y(:,10));
% plot(t, (y(:,9)/1.67e-3));

xlabel('time [s]');
ylabel('\ith [kg*m/s]', 'FontSize', 11);
txt = {'\ith \rm= 0.003 \it[kg*m/s]','\rm\downarrow', '\rm\bullet'};
text(2400, 0.00385,txt,'FontSize', 11);
% text(2000, 0.0028,txt,'FontSize', 11);
title('Angular Momentum of Reaction Wheels','FontSize', 20);
yline(0, 'k');
xline(settleTime, 'b--', ['x = ', num2str(round(settleTime,2))]);
for i=1:length(orbitPeriods)
    xstr = num2str(orbitPeriods(i))+ "Orbit(s)  ";
    xline(orbitPeriods(i)*orbitPeriod, 'k:', xstr);
end
legend("X-axis", "Y-axis","Z-axis",'Settling Time', 'Orbit(s)','FontSize', 11);
hold off

% figure(8);
% plot(torque(:,1), torque(:,2));
% hold on
% plot(torque(:,1), torque(:,3));
% plot(torque(:,1), torque(:,4));
% xlabel('time [s]');
% ylabel('\ith [kg*m^{2}/s]', 'FontSize', 11);
% txt = {'\ith \rm= 0.003 \it[Nm]','\rm\downarrow', '\rm\bullet'};
% text(2400, 0.00385,txt,'FontSize', 11);
% title('Torque of Reaction Wheels','FontSize', 20);
% xline(settleTime, 'b--', ['x = ', num2str(round(settleTime,2))]);
% for i=1:length(orbitPeriods)
%     xstr = num2str(orbitPeriods(i))+ "Orbit(s)  ";
%     xline(orbitPeriods(i)*orbitPeriod, 'k:', xstr);
% end
% legend("X-axis", "Y-axis","Z-axis",'Settling Time', 'Orbit(s)','FontSize', 11);
% hold off

fprintf("Finished\n");
