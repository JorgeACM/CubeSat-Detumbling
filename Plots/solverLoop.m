% Script: solver(t)
% 2023_24
clear
clc

% Setup the global variables
global v1 v2 v3 control ms md wd wp qtg,
v1 = 0;
v2 = 0;
qt = [0,0,0,1]';
% Set the simulation duration
% t_max = 100;
% Orbital Period = 5832.911569932271
n = sqrt(3.98e5/7000^3);
orbitPeriod = 2*pi()/n;
orbits  = 4;
t_max = orbits*5833;
% t_max=200;

orbitPeriods=1:3:orbits;

yend = [0 0 1 2 3];
v3=146;
% https://aerospace-europe.eu/media/books/delft-0025.pdf
% a = 0;
% for v3 = 135:.2:141

% control = "magnetorquer1";
control = "magnetorquer2";
% control = "wheels";

% Settling Time Parameters

% yfinal = [0,0,0,1, 0, -n,0,0,0,0]';
yfinal = [0, -n,0, n, -1, 1]';
yinit = [1, 2, 3, norm([1,2,3]), 1, 1]';

% for v3 = 135:1:141
% times = [kd, ks, STx, STy, STz, STnorm]
ST =[];
% ms=1000;
% md=0.1;
% kds=[996, 0.06; 996, 0.07; 1000, 0.1; 1000, .06];
kds = [992, 0.06; 994, 0.06; 996, 0.06; 1000, 0.06;996, 0.07; 1000, 0.1; 990, 0.06];
% wdd=[5,12; 6,12; 7,12; 8,12; 9,12; 10,12; 11,12; 12,12; 13,12;14,12;15,12];
k=1;
% for ms = 988:2:1000
%     for md = 0.01:.01:.12
for i=1:length(kds)
        ms=kds(i,1);
        md=kds(i,2);
        fprintf('(%d, %.2f)\n', ms, md);
% for i=1:250
    % v3=i;
    % disp(i);
% for i=1:length(wdd)
%         wd=wdd(i,1);
%         wp=wdd(i,2);
%         fprintf('(%d, %.2f)\n', wd, wp);
% for wd = 9:1:12
%     for wp = 5:1:10
%         fprintf('(%d, %d)\n', wd, wp);
    norms=[];
    vel=[];
    % % Simulate the attitude of the spacecraft for t seconds duration
    [t y] = ode45(@attitude_model, [0 t_max], [0 0 0 1 1 2 3 0 0 0]);

    % % Norm of Angular Velocities
    for j = 1:size(y,1)
        norms(j) = norm(y(j,5:7));
    end
    vel = cat(2, y(:,5:7), norms',y(:,4), y(:,4));

    % % Calculate Settling Time
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
    ST = cat(1,ST,[md, ms, STx, STy, STz, STnorm, STq, STq2, STtotal]);
    % ST = cat(1,ST,[v3, STx, STy, STz, STnorm, STq, STq2, STtotal]);
    % ST = cat(1,ST,[wd, wp, STx, STy, STz, STnorm, STq, STq2,STtotal]);
    
    % figure(k);
    % plot(t, y(:,5));
    % hold on;
    % plot(t, y(:,6));
    % plot(t, y(:,7));
    % 
    % xlabel('time [s]');
    % ylabel('Angular Velocities  [rad/s]');
    % xline(STtotal, 'b--', ['x = ', num2str(STtotal)]);
    % for u=1:length(orbitPeriods)
    %     xline(orbitPeriods(u)*orbitPeriod, 'k:', [num2str(orbitPeriods(u)), ' O P'])
    % end
    % legend("\omega_{x}", "\omega_{y}","\omega_{z}", "Settling Time","Orbital Periods",'FontSize', 14);
    % titlestr = "Rotation Rates in Body Frame, Kd = "+num2str(wd)+", Kp = "+ num2str(wp)+".";
    % title(titlestr,'fontsize', 20);
    % hold off
    % k=k+1;
    % 
    % figure(k);
    % plot(t, y(:,1))
    % hold on
    % plot(t, y(:,2))
    % plot(t, y(:,3))
    % plot(t, y(:,4))
    % 
    % xlabel('time [s]');
    % ylabel('quaternion');
    % xline(STtotal, 'b--', ['x = ', num2str(STtotal)]);
    % for u=1:length(orbitPeriods)
    %     xline(orbitPeriods(u)*orbitPeriod, 'k:', [num2str(orbitPeriods(u)), ' O P']);
    % end
    % legend("q1", "q2","q3","q4", "Orbital Periods");
    % titlestr = "Quaternions, Kd = "+num2str(wd)+", Kp = "+ num2str(wp)+".";
    % title(titlestr,'fontsize', 20);
    % hold off
    % k=k+1;

    % end
end




% Norm of Angular Velocities
% for i = 1:size(y,1)
%     magnitudes(i) = norm(y(i,5:7));
% end


fprintf('Finished Loop...');
fprintf('value of control: ');
disp(control)
% mat = [ST(:,1:4), ST(:,end)];
mat = [ST(:,1:5), ST(:,end)];
% mat = [ST(:,1:5), ST(:,7),ST(:,end)];
ST_NaN = mat(~any(isnan(mat), 2), :); 
if control == "magnetorquer2"
    kd=ST_NaN(:,1);
    ks=ST_NaN(:,2);
    wx=ST_NaN(:,3);
    wy=ST_NaN(:,4);
    wz=ST_NaN(:,5);
    wt=ST_NaN(:,6);
    wtnorm=ST_NaN(:,6)/3600;
    wtorbit=ST_NaN(:,6)/(2*pi()/n);
    wtotal=ST_NaN(:,end);
    wtotalhrs=ST_NaN(:,end)/3600;
    wtotalorbit=ST_NaN(:,end)/(2*pi()/n);
    [X, Y] = meshgrid(min(kd):0.01:max(kd), min(ks):1:max(ks));
    Z = griddata(kd, ks, wtotalhrs, X, Y);
    figure(k);
    surf(X,Y,Z);
    xlabel('Kd Gain','FontSize', 14, 'FontWeight', 'bold');
    ylabel('Ks Gain','FontSize', 14, 'FontWeight', 'bold');
    zlabel('Settling time [hrs]','FontSize', 14, 'FontWeight', 'bold');
    title('Settling Time in Hours of Rotation Rates with different P-D Gains.','FontSize', 20);
    k=k+1;

    Z = griddata(kd, ks, wtotal, X, Y);
    figure(k);
    surf(X,Y,Z);
    xlabel('Kd Gain','fontsize', 14, 'FontWeight', 'bold');
    ylabel('Ks Gain','fontsize', 14, 'FontWeight', 'bold');
    zlabel('Settling time [s]','fontsize', 14, 'FontWeight', 'bold');
    title('Settling Time in Seconds of Rotation Rates with different P-D Gains.','fontsize', 16);
    k=k+1;

    Z = griddata(kd, ks, wtotalorbit, X, Y);
    figure(k);
    surf(X,Y,Z);
    xlabel('Kd Gain','fontsize', 14, 'FontWeight', 'bold');
    ylabel('Ks Gain','fontsize', 14, 'FontWeight', 'bold');
    zlabel('Settling time [Orbital Periods]','fontsize', 14, 'FontWeight', 'bold');
    title('Settling Time in Orbital Periods of Rotation Rates with different P-D Gains.','fontsize', 16);
    k=k+1;
elseif control == "magnetorquer1"
    % [v3, STx, STy, STz, STnorm, STq, STq2]
     kp=ST_NaN(:,1);
     wx=ST_NaN(:,2); wy=ST_NaN(:,3); wz=ST_NaN(:,4);
     wt=ST_NaN(:,end); wthrs=ST_NaN(:,end)/3600; wtorbit=ST_NaN(:,end)/(2*pi()/n);
     STmean=mean(wt)
     figure(k);
     plot(kp,wt)
     xlabel('K Gain','fontsize', 14, 'FontWeight', 'bold');
     ylabel('Settling time [seconds]','fontsize', 14, 'FontWeight', 'bold');
     title('Settling Time in seconds of Rotation Rates for different Proportional Gains.','fontsize', 14);
     k=k+1;

     figure(2);
     plot(kp,wthrs)
     xlabel('K Gain','FontSize', 14, 'FontWeight', 'bold');
     ylabel('Settling time [hours]','FontSize', 14, 'FontWeight', 'bold');
     txt = {'\bullet', '\uparrow', 'K = 146'};
     text(145.9, 5.653,txt,'FontSize', 11);
     title('Settling Time in Hours of Rotation Rates for different Proportional Gains.','FontSize', 20);
     k=k+1;

     figure(k);
     plot(kp,wtorbit)
     xlabel('K Gain','fontsize', 14, 'FontWeight', 'bold');
     ylabel('Settling time [Orbital Periods]','fontsize', 14, 'FontWeight', 'bold');
     title('Settling Time in Orbital Periods of Rotation Rates for different Proportional Gains.','fontsize', 14);
     k=k+1;

     figure(k);
     plot(kp,wy)
     xlabel('K Gain','fontsize', 14, 'FontWeight', 'bold');
     ylabel('Settling time in hours','fontsize', 14, 'FontWeight', 'bold');
     title('Settling Time in Hours of Y-Axis Rotation Rate for different Proportional Gains.','fontsize', 14);
     k=k+1;

elseif control == "wheels"
    fprintf("Executing IF_ELSE Wheel...");
    % k=1;
    % [wd, wp, STx, STy, STz, STnorm, STq, STq2,STtotal]
    kd=ST_NaN(:,1);
    ks=ST_NaN(:,2);
    wx=ST_NaN(:,3);
    wy=ST_NaN(:,4);
    wz=ST_NaN(:,5);
    wnorm=ST_NaN(:,6);
    qt=ST_NaN(:,7);
    STtotal=ST_NaN(:,end);
    STtotalmin=ST_NaN(:,end)/60;
    [X, Y] = meshgrid(min(kd):0.01:max(kd), min(ks):1:max(ks));
    Z = griddata(kd, ks, STtotalmin, X, Y);
    figure(k);
    
    ss1= surf(X,Y,Z);
    ss1.EdgeColor = 'none';
    colorbar
    xlabel('Kd Gain','fontsize', 14, 'FontWeight', 'bold');
    ylabel('Ks Gain','fontsize', 14, 'FontWeight', 'bold');
    zlabel('Settling time [min]','fontsize', 14, 'FontWeight', 'bold');
    title('Settling Time in Minutes of Rotation Rates with different P-D Gains.','fontsize', 16);
    k=k+1;

    Z = griddata(kd, ks, STtotal, X, Y);
    figure(k);
    ss2 = surf(X,Y,Z);
    ss2.EdgeColor = 'none';
    colorbar
    xlabel('Kd Gain','fontsize', 14, 'FontWeight', 'bold');
    ylabel('Ks Gain','fontsize', 14, 'FontWeight', 'bold');
    zlabel('Settling time [s]','fontsize', 14, 'FontWeight', 'bold');
    title('Settling Time in Seconds of Rotation Rates with different P-D Gains.','fontsize', 16);
    k=k+1;

end

fprintf("Hello");


% settleTime = max(ST(end,3:5));
% 
% figure(k);
% plot(t, y(:,5));
% hold on;
% plot(t, y(:,6));
% plot(t, y(:,7));
% 
% xlabel('time [s]','fontsize', 14);
% ylabel('Rotation Rates  [rad/s]','fontsize', 14);
% xline(settleTime, 'b--', ['x = ', num2str(round(settleTime,2))]);
% for i=1:length(orbitPeriods)
%     xstr = num2str(orbitPeriods(i))+ " O P";
%     xline(orbitPeriods(i)*orbitPeriod, 'k:', xstr);
% end
% title('Rotation Rates in Body Frame','fontsize', 20);
% legend("X-axis", "Y-axis","Z-axis", "Settle Time","Orbital Periods",'fontsize', 12);
% hold off
% k=k+1;
% 
% figure(k);
% plot(t, norms');
% hold on;
% xlabel('time');
% ylabel('rad/s');
% xline(STnorm, 'r--');
% xline(S(4).TransientTime, 'b--');
% text(STnorm+0.5, 0.5, ['x = ',num2str(STnorm)], 'Color', 'red');
% text(S(4).TransientTime+0.1, 0.8, ['x = ',num2str(S(4).TransientTime)], 'Color', 'blue');
% 
% title('Normalized rotation');
% for i=1:length(orbitPeriods)
%     xline(orbitPeriods(i)*orbitPeriod, 'k:', [num2str(orbitPeriods(i)), " O P"]);
% end
% legend('Wnorm', 'Settling Time', 'Transient Time', "Orbital Periods");
% hold off;
% k=k+1;
% 
% figure(k);
% plot(t, y(:,5));
% hold on;
% xlabel('time');
% ylabel('rad/s');
% xline(STx, 'r--');
% xline(S(1).TransientTime, 'b--');
% text(STx+0.5, 0.5, ['x = ',num2str(STx)], 'Color', 'red');
% text(S(1).TransientTime+0.1, 0.8, ['x = ',num2str(S(1).TransientTime)], 'Color', 'blue');
% 
% title('x-axis rotation');
% for i=1:length(orbitPeriods)
%     xline(orbitPeriods(i)*orbitPeriod, 'k:', [num2str(orbitPeriods(i)), ' O P']);
% end
% legend('Wx', 'Settling Time', 'Transient Time', "Orbital Periods");
% hold off;
% k=k+1;
% 
% figure(k);
% plot(t, y(:,6));
% hold on;
% xlabel('time');
% ylabel('rad/s');
% xline(STy, 'r--');
% xline(S(2).TransientTime, 'b--');
% text(STy+0.5, 0.5, ['x = ',num2str(STy)], 'Color', 'red');
% text(S(2).TransientTime+0.1, 0.8, ['x = ',num2str(S(2).TransientTime)], 'Color', 'blue');
% 
% title('Y-axis rotation');
% for i=1:length(orbitPeriods)
%     xline(orbitPeriods(i)*orbitPeriod, 'k:', [num2str(orbitPeriods(i)), ' O P']);
% end
% legend('Wy', 'Settling Time', 'Transient Time', "Orbital Periods");
% hold off;
% k=k+1;
% 
% figure(k);
% plot(t, y(:,7));
% hold on;
% xlabel('time');
% ylabel('rad/s');
% xline(STz, 'r--');
% xline(S(3).TransientTime, 'b--');
% text(STz+0.5, 0.5, ['x = ',num2str(STz)], 'Color', 'red');
% text(S(3).TransientTime+0.1, 0.8, ['x = ',num2str(S(3).TransientTime)], 'Color', 'blue');
% 
% title('Z-axis rotation');
% for i=1:length(orbitPeriods)
%     xline(orbitPeriods(i)*orbitPeriod, 'k:', [num2str(orbitPeriods(i)), ' O P']);
% end
% legend('Wz', 'Settling Time', 'Transient Time', "Orbital Periods");
% hold off;
% k=k+1;

% % Plot the results
% figure(1);
% % hold on
% plot(t, y(:,5));
% % plot(t, rad2deg(y(:,5)));
% xlabel('time [s]');
% ylabel('x-axis rotation  [rad/s]');
% % ylabel('x-axis rotation [deg/s]');
% % Update legend
% % legend_labels{v3 - 134} = ['y=' num2str(v3)];
% % end
% 
% % Add legend
% % legend(legend_labels);
% % xlabel('time [s]');
% % ylabel('x-axis rotation [rad/s]');
% % [settling_time,settling_period] = calculate_settling_time(t, y(:,6));
% 
% figure(2);
% plot(t, y(:,6));
% % plot(t, rad2deg(y(:,6)));
% xlabel('time [s]');
% ylabel('y-axis rotation  [rad/s]');
% % ylabel('y-axis rotation  [deg/s]');
% 
% figure(3);
% plot(t, y(:,7));
% % plot(t, rad2deg(y(:,7)));
% xlabel('time [s]');
% ylabel('z-axis rotation  [rad/s]');
% % ylabel('z-axis rotation  [deg/s]');
% 
% figure(4);
% plot(t, y(:,1))
% hold on
% plot(t, y(:,2))
% plot(t, y(:,3))
% plot(t, y(:,4))
% legend("q1", "q2","q3","q4")
% xlabel('time [s]');
% ylabel('quaternion');
% hold off
% 
% figure(5);
% plot(t, y(:,8));
% hold on
% plot(t, y(:,9));
% plot(t, y(:,10));
% % plot(t, (y(:,9)/1.67e-3));
% legend("x-axis", "y-axis","z-axis");
% xlabel('time [s]');
% ylabel('Angular Momentum  [Nm]');
% hold off

% figure(7)
% surf(ST(:,1),ST(:,2),ST(:,3));
% colorbar
% xlabel('Kd');
% ylabel('Ks');
% zlabel('Settling time');
% title('X-axis Angular Velocity Settle Times with different Gains.');
% 
% figure(8)
% surf(ST(:,1),ST(:,2),ST(:,4));
% colorbar
% xlabel('Kd');
% ylabel('Ks');
% zlabel('Settling time');
% title('Y-axis Angular Velocity Settle Times with different Gains.');
% 
% figure(9)
% surf(ST(:,1),ST(:,2),ST(:,5));
% colorbar
% xlabel('Kd');
% ylabel('Ks');
% zlabel('Settling time');
% title('Z-axis Angular Velocity Settle Times with different Gains.');
% 
% figure(10)
% surf(ST(:,1),ST(:,2),ST(:,6));
% 
% colorbar
% xlabel('Kd');
% ylabel('Ks');
% zlabel('Settling time W');
% title('Angular Velocity Norm Settle Times with different Gains.');

