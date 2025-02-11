% Script: solver(t)
% 2023_24
clear
clc

% Setup the global variables
global v1 v2 v3 control ms md wd wp qtg,

fprintf("Starting...\n");
% Set the simulation duration
% t_max = 100;
orbits = 1;
t_max = orbits*5833;

% % Set an array of time of each orbit period
n = sqrt(3.98e5/7000^3);
orbitPeriod = 2*pi()/n;
orbitPeriods= 1:1:orbits;


% % Select Algorithm
% control = "magnetorquer1";
% control = "magnetorquer2";
control = "wheels";

% % Parameters for calculate Settling Time
% % Parameters [wx wy wz wnorm qt qt]
yinit = [1, 2, 3, norm([1,2,3]), 1, 1]';    % Initial Values
yfinal = [0, -n, 0, n, -1, 1]';              % Stabilized Values

% % Gains
% % Magnetometer #1
v3 = 146;
% % Magnetometer #2
ms=990;
md=0.06;
% % Reaction Wheels
wd= 9;
wp = 5;


% % The next loops are used to test the controllers with several gains to 
% % find the best combination of gains.
% % Just one algorithm can be used at a time
% % Each gain global variable in control_algorihm.m file must be
% % uncommented

% % Array to Store Settling times
ST=[];

fprintf("Starting loop...\n");
% % % %  M A G N E T O R Q U E R  # 1   
% % Loop to try different B-dot ~ k (w x B)
% for i=1:250
%     v3=i;
%     disp(i);

% % % %  M A G N E T O R Q U E R  # 2   Ks   Kd
% % Loop to try different Magnetorquers PD Gains
% for ms=980:1:1010
%     for md=0.01:0.01:.15
%         fprintf('(%d, %.2f)\n', ms, md);

% % % R E A C T I O N    W H E E L S
% % Loop to try different PD Gain for Reaction Wheels
% for wd = 9:1:12
%     for wp = 5:1:10
%         fprintf('(%d, %d)\n', wd, wp);
        
        % Simulate the attitude of the spacecraft for t seconds duration
        [t y] = ode45(@attitude_model, [0 t_max], [0 0 0 1 1 2 3 0 0 0]);

        % % Array to store Normalized Angular Velocities of all iterations
        norms=[];
        for j = 1:size(y,1)
            norms(j) = norm(y(j,5:7));
        end
        
        % Array to store values of y from different iterations
        % vel = [wx wy wz wnorm qt qt]
        vel=[];
        vel = cat(2, y(:,5:7), norms',y(:,4), y(:,4));

        % Calculate Settling Time
        S = lsiminfo(vel(2:end,:),t(2:end), yfinal, yinit);
        % S = lsiminfo(vel,t, yfinal, yinit,'SettlingTimeThreshold',0.05);
        % S = lsiminfo(vel,t, 'SettlingTimeThreshold',0.02);
        STx = S(1).SettlingTime;
        STy = S(2).SettlingTime;
        STz = S(3).SettlingTime;
        STnorm = S(4).SettlingTime;
        STq = S(5).SettlingTime;
        STq2 = S(6).SettlingTime;
        STtotal= max([STx, STy, STz]);
        % STtotal= max([STx, STy, STz, STq]);

        % % Settling times of Magnetometer #1 B-dot Controller
        % ST = cat(1,ST,[v3, STx, STy, STz, STnorm, STq, STq2, STtotal]);

        % % Settling times of Magnetometer #2 PD (B-dot) Controller
        % ST = cat(1,ST,[md, ms, STx, STy, STz, STnorm, STq, STq2, STtotal]);
        
        % % Settling times of Reaction Wheel PD Controller
        ST = cat(1,ST,[wd, wp, STx, STy, STz, STnorm, STq, STq2,STtotal]);


% % Uncomment last "end" when Magnetometer #1 selected
% % Uncomment both "end"s when Magnetometer #2 or Reaction Wheel Controller selected

%     end
% end
% fprintf('Loop Finished...\n');


% Plot the results
settleTime=ST(end);
% % &&&&&&&&&&&&&&&&&&
% % Plot Angular Velocities
% % X-axis
figure(1);
subplot(3,1,1);
plot(t, y(:,5));
hold on;
xlabel('time [s]');
ylabel('rad/s');
xline(S(1).TransientTime, 'r--');
text(S(1).TransientTime+0.1, 0.8, ['x = ',num2str(S(1).TransientTime)], 'Color', 'red');
xline(STx, 'b--');
text(STx+0.5, 0.5, ['x = ',num2str(STx)], 'Color', 'blue');
for i=1:length(orbitPeriods)
    xstr = num2str(orbitPeriods(i))+ " O P";
    xline(orbitPeriods(i)*orbitPeriod, 'k:', xstr);
end
legend('Wx', 'Transient Time', 'Settling Time', "Orbital Periods",'FontSize', 12);
title('X-axis rotation');
hold off;

% % Y-axis
subplot(3,1,2);
plot(t, y(:,6));
hold on;
xlabel('time [s]');
ylabel('rad/s');
xline(STy, 'b--');
text(STy+0.5, 0.5, ['x = ',num2str(STy)], 'Color', 'blue');
xline(S(2).TransientTime, 'r--');
text(S(2).TransientTime+0.1, 0.8, ['x = ',num2str(S(2).TransientTime)], 'Color', 'red');
for i=1:length(orbitPeriods)
    xstr = num2str(orbitPeriods(i))+ " O P";
    xline(orbitPeriods(i)*orbitPeriod, 'k:', xstr);
end
legend('Wy', 'Transient Time', 'Settling Time',"Orbital Periods",'FontSize', 12);
title('Y-axis rotation');
hold off;

% % Z-axis
subplot(3,1,3);
plot(t, y(:,7));
hold on;
xlabel('time [s]');
ylabel('rad/s');
xline(S(3).TransientTime, 'r--');
text(S(3).TransientTime+0.1, 0.8, ['x = ',num2str(S(3).TransientTime)], 'Color', 'red');
xline(STz, 'b--');
text(STz+0.5, 0.5, ['x = ',num2str(STz)], 'Color', 'blue');
for i=1:length(orbitPeriods)
    xstr = num2str(orbitPeriods(i))+ " O P";
    xline(orbitPeriods(i)*orbitPeriod, 'k:', xstr);
end
legend('Wz', 'Transient Time', 'Settling Time', "Orbital Periods",'FontSize', 12);
title('Z-axis rotation');
hold off;

% % Three axis
figure(2);
% Plot normal and Zoom version
% subplot(2,1,1);
% xlim([0,45]); ylim([-.3, 3]);
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
% % Generic Title
title('Rotation Rates in Body Frame','FontSize', 20);
% % Title when selecting Reaction Wheels
% title({'Reaction Wheel Controller Rotation Rates in Body Frame Kd = 9, Kp = 5'},'FontSize', 20);
legend("X-axis", "Y-axis","Z-axis", "Settling Time","Orbital Periods",'FontSize', 12);
% legend("\omega_{x}", "\omega_{y}","\omega_{z}", "Settling Time","Orbital Periods",'FontSize', 14);
hold off

% % Plot Zoom Verison of Angular Velocities
% subplot(2,1,2);
% plot(t, y(:,5));
% hold on;
% plot(t, y(:,6));
% plot(t, y(:,7));
% xlabel('time [s]','FontSize', 14);
% ylabel('Rotation Rates  [rad/s]','FontSize', 14);
% xline(settleTime, 'b--', ['x = ', num2str(round(settleTime,2))], 'FontSize', 11);
% for i=1:length(orbitPeriods)
%     xstr = num2str(orbitPeriods(i))+ " O P";
%     xline(orbitPeriods(i)*orbitPeriod, 'k:', xstr);
% end
% ylim([-.005, .005]);
% xlim([0,45]);
% txt = {'\rm\bullet', '\rm\uparrow','\omega_{y} ≈ -0.0011 rad/s', };
% text(35, -.0018,txt,'FontSize', 12);
% txt1 = {'\omega_{x} ≈ \omega_{z} ≈ 0 rad/s', '\rm\downarrow', '\rm\bullet',};
% text(37.5, 8e-4,txt1,'FontSize', 12);
% title('Rotation Rates in Body Frame Zoom-in','FontSize', 18);
% legend("X-axis", "Y-axis","Z-axis", "Settling Time","Orbital Periods",'FontSize', 12);
% % legend("\omega_{x}", "\omega_{y}","\omega_{z}", "Settling Time","Orbital Periods",'FontSize', 14);
% hold off

% % Quaternions
figure(3);
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

figure(4);
plot(t, y(:,8));
hold on
plot(t, y(:,9));
plot(t, y(:,10));
xlabel('time [s]');
ylabel('\ith [kg*m/s]', 'FontSize', 11);
% txt = {'\ith \rm= 0.003 \it[kg*m/s]','\rm\downarrow', '\rm\bullet'};
% text(2400, 0.00385,txt,'FontSize', 11);
% text(2000, 0.0028,txt,'FontSize', 11);
title('Angular Momentum of Reaction Wheels','FontSize', 20);
xline(settleTime, 'b--', ['x = ', num2str(round(settleTime,2))]);
for i=1:length(orbitPeriods)
    xstr = num2str(orbitPeriods(i))+ "Orbit(s)  ";
    xline(orbitPeriods(i)*orbitPeriod, 'k:', xstr);
end
legend("X-axis", "Y-axis","Z-axis",'Settling Time', 'Orbit(s)','FontSize', 11);
hold off

% % 3D Plots to compare Gains responses

% % mat = [ST(:,1:4), ST(:,end)];             % Magnetorquer #1
% % mat = [ST(:,1:5), ST(:,end)];               % Magnetorquer #2
% mat = [ST(:,1:5), ST(:,7),ST(:,end)];     % Reaction Wheels
% ST_NaN = mat(~any(isnan(mat), 2), :); 
% if control == "magnetorquer2"
%     kd=ST_NaN(:,1);
%     ks=ST_NaN(:,2);
%     wx=ST_NaN(:,3);
%     wy=ST_NaN(:,4);
%     wz=ST_NaN(:,5);
%     wt=ST_NaN(:,6);
%     wtnorm=ST_NaN(:,6)/3600;
%     wtorbit=ST_NaN(:,6)/(2*pi()/n);
%     wtotal=ST_NaN(:,end);
%     wtotalhrs=ST_NaN(:,end)/3600;
%     wtotalorbit=ST_NaN(:,end)/(2*pi()/n);
%     [X, Y] = meshgrid(min(kd):0.01:max(kd), min(ks):1:max(ks));
%     Z = griddata(kd, ks, wtotalhrs, X, Y);
% 
%     figure(101);
%     surf(X,Y,Z);
%     xlabel('Kd Gain','FontSize', 14, 'FontWeight', 'bold');
%     ylabel('Ks Gain','FontSize', 14, 'FontWeight', 'bold');
%     zlabel('Settling time [hrs]','FontSize', 14, 'FontWeight', 'bold');
%     title('Settling Time in Hours of Rotation Rates with different P-D Gains.','FontSize', 20);
% 
%     Z = griddata(kd, ks, wtotal, X, Y);
%     figure(102);
%     surf(X,Y,Z);
%     xlabel('Kd Gain','fontsize', 14, 'FontWeight', 'bold');
%     ylabel('Ks Gain','fontsize', 14, 'FontWeight', 'bold');
%     zlabel('Settling time [s]','fontsize', 14, 'FontWeight', 'bold');
%     title('Settling Time in Seconds of Rotation Rates with different P-D Gains.','fontsize', 16);
% 
%     Z = griddata(kd, ks, wtotalorbit, X, Y);
%     figure(103);
%     surf(X,Y,Z);
%     xlabel('Kd Gain','fontsize', 14, 'FontWeight', 'bold');
%     ylabel('Ks Gain','fontsize', 14, 'FontWeight', 'bold');
%     zlabel('Settling time [Orbital Periods]','fontsize', 14, 'FontWeight', 'bold');
%     title('Settling Time in Orbital Periods of Rotation Rates with different P-D Gains.','fontsize', 16);
% 
% elseif control == "magnetorquer1"
%     % [v3, STx, STy, STz, STnorm, STq, STq2]
%      kp=ST_NaN(:,1);
%      wx=ST_NaN(:,2); wy=ST_NaN(:,3); wz=ST_NaN(:,4);
%      wt=ST_NaN(:,end); wthrs=ST_NaN(:,end)/3600; wtorbit=ST_NaN(:,end)/(2*pi()/n);
%      STmean=mean(wt);
% 
%      figure(201);
%      plot(kp,wt)
%      xlabel('K Gain','fontsize', 14, 'FontWeight', 'bold');
%      ylabel('Settling time [seconds]','fontsize', 14, 'FontWeight', 'bold');
%      title('Settling Time in seconds of Rotation Rates for different Proportional Gains.','fontsize', 14);
% 
% 
%      figure(202);
%      plot(kp,wthrs)
%      xlabel('K Gain','FontSize', 14, 'FontWeight', 'bold');
%      ylabel('Settling time [hours]','FontSize', 14, 'FontWeight', 'bold');
%      txt = {'\bullet', '\uparrow', 'K = 146'};
%      text(145.9, 5.653,txt,'FontSize', 11);
%      title('Settling Time in Hours of Rotation Rates for different Proportional Gains.','FontSize', 20);
% 
% 
%      figure(203);
%      plot(kp,wtorbit)
%      xlabel('K Gain','fontsize', 14, 'FontWeight', 'bold');
%      ylabel('Settling time [Orbital Periods]','fontsize', 14, 'FontWeight', 'bold');
%      title('Settling Time in Orbital Periods of Rotation Rates for different Proportional Gains.','fontsize', 14);
% 
% 
%      figure(204);
%      plot(kp,wy)
%      xlabel('K Gain','fontsize', 14, 'FontWeight', 'bold');
%      ylabel('Settling time in hours','fontsize', 14, 'FontWeight', 'bold');
%      title('Settling Time in Hours of Y-Axis Rotation Rate for different Proportional Gains.','fontsize', 14);
% 
% 
% elseif control == "wheels"
%     fprintf("Executing IF_ELSE Wheel...");
%     % k=1;
%     % [wd, wp, STx, STy, STz, STnorm, STq, STq2,STtotal]
%     kd=ST_NaN(:,1);
%     ks=ST_NaN(:,2);
%     wx=ST_NaN(:,3);
%     wy=ST_NaN(:,4);
%     wz=ST_NaN(:,5);
%     wnorm=ST_NaN(:,6);
%     qt=ST_NaN(:,7);
%     STtotal=ST_NaN(:,end);
%     STtotalmin=ST_NaN(:,end)/60;
%     [X, Y] = meshgrid(min(kd):0.01:max(kd), min(ks):1:max(ks));
%     Z = griddata(kd, ks, STtotalmin, X, Y);
% 
%     figure(301);
%     ss1= surf(X,Y,Z);
%     ss1.EdgeColor = 'none';
%     colorbar
%     xlabel('Kd Gain','fontsize', 14, 'FontWeight', 'bold');
%     ylabel('Ks Gain','fontsize', 14, 'FontWeight', 'bold');
%     zlabel('Settling time [min]','fontsize', 14, 'FontWeight', 'bold');
%     title('Settling Time in Minutes of Rotation Rates with different P-D Gains.','fontsize', 16);
% 
% 
%     Z = griddata(kd, ks, STtotal, X, Y);
%     figure(302);
%     ss2 = surf(X,Y,Z);
%     ss2.EdgeColor = 'none';
%     colorbar
%     xlabel('Kd Gain','fontsize', 14, 'FontWeight', 'bold');
%     ylabel('Ks Gain','fontsize', 14, 'FontWeight', 'bold');
%     zlabel('Settling time [s]','fontsize', 14, 'FontWeight', 'bold');
%     title('Settling Time in Seconds of Rotation Rates with different P-D Gains.','fontsize', 16);
% 
% 
% end

fprintf("THE END.\n")