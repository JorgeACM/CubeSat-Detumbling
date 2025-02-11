% Function: [M T] = control_algorithm(t, q, w, h)
% Programmed by: Jorge Chavarin
% Date: 09/04/2024

function [M T] = control_algorithm(t, q, w, h, Bb)

% Inputs:
%    t = time
%    q = attitude quaternion with respect to the orbital frame (4-element vector)
%    w = rotation rates in body frame (3-element vector)
%    h = accumulated momentum in the wheels (3-element vector)
%    Bb = magnetic field in body frame (3-element vector)

% Outputs:
%    M = commanded magnetic dipole moment (3-element vector)
%    T = commanded torque for the momentum wheels (3-element vector)

% Global variable: user defined... 
global v1 v2 v3 control  ms md wp wd qtg

% Default output
M = [0 0 0]';
T = [0 0 0]';
n = sqrt(3.98e5/7000^3); % orbital angular velocity
% control = "magnetorquer1";
% control = "magnetorquer2";
% control = "wheels";


if control == "magnetorquer1"
    % % Bdot ~ k (w x B)
    m=cross(w,Bb);
    % Controller Gain
    k=v3; % Global variable used for tunning
    % k=146;
    M = (k*m)';
    
elseif control == "magnetorquer2"
% Activate both algorithm depending on the angular velocity of the satellite
% if abs(w(1)) > .1 || abs(w(2)) > 1.05*n || abs(w(3)) > .1
        
    % % Magnetic field in body frame
    Bx = Bb(1);
    By = Bb(2);
    Bz = Bb(3);
    % B = sqrt(Bx^2 + By^2 + Bz^2);
    B = norm(Bb);
    
    
    % % Angle between satellite Y axis and magnetic field vector
    Betha = acos(By/B);

    % % Derivative of B is equal to difference of Betha at actual time t 
    % % and Betha at previous time t.
    dB = (Betha-v1);
    v1 = Betha;

    % % Time difference.
    dT = t - v2;
    v2 = t;
    % % Derivative of Betha with respect to time
    Bdot = dB/dT;


    % % Angular velocities
    wy = w(2);
    wref = -n;
    
    My=0; Mx =0; Mz = 0;

    % % System Gain Variables
    % % Using global variables for tunning
    Ks=ms; Kd=md;

    % % First approach
    % Ks = 1000; Kd = 0.1;
    
    % Ks=990;
    % Kd=0.06;

    if dT>0
        % % B-dot Controller
        My =Kd * Bdot;
        % if abs(Bz) > abs(Bx)
            Mx = Ks*(wy-wref)*sign(Bz);
        % end
        % if abs(Bx) > abs(Bz)
        %     Mz = -Ks*(wy-wref)*sign(Bx);
        % end
        
    end
    
    M = [Mx, My, Mz]';
elseif control == "wheels"
% else
    
    % % Target Quaternion
    % qt = qtg; % Using global variable qtg for simulation purposes
    qt = [0,0,0,1]';

    % % Target Quaternion Normalized
    scale = sqrt(qt(1)^2 + qt(2)^2 + qt(3)^2 + qt(4)^2);
    qt(1) = qt(1)/scale;
    qt(2) = qt(2)/scale;
    qt(3) = qt(3)/scale;
    qt(4) = qt(4)/scale;
    
    % % Rotation Matrix from Actual orientation to desired one.
    Aqt = [ qt(4),  qt(3), -qt(2), qt(1);
           -qt(3),  qt(4),  qt(1), qt(2);
            qt(2), -qt(1),  qt(4), qt(3);
           -qt(1), -qt(2), -qt(3), qt(4)];

    % % Quaternion Error calculation
    qe = Aqt*[-q(1); -q(2); -q(3); q(4)];
    
    % % &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    % First version of Reaction Wheel Controller
    % Not used
    % Kx = 1; Ky=1; Kz = 1;
    % Kxd=1; Kyd=1; Kzd = 1;
    % 
    % Tx = 2*Kx*qe(1)*qe(4) + Kxd*w(1);
    % Ty = 2*Ky*qe(2)*qe(4) + Kyd*w(2);
    % Tz = 2*Kz*qe(3)*qe(4) + Kzd*w(3);

    % Tx =0; Ty = Tx; Tz = Ty;
    % Kpy = 5;
    % Kdy = 1;
    % 
    % Ty = Kpy*( sin( q(2)*sign( q(4) ) )^-1 ) + Kdy*w(2);
    % T = [Tx, Ty, Tz]';
    % % &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    % % Matrix of Inertia
    I = [1.67e-3 0 0;
     0 1.67e-3 0;
     0 0 1.67e-3];

    % % Rotation Matrix
    % % From body frame to orbit frame
    Aob = [q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2 2*(q(1)*q(2) + q(4)*q(3)) 2*(q(1)*q(3) - q(4)*q(2));
       2*(q(1)*q(2) - q(4)*q(3)) -q(1)^2 + q(2)^2 - q(3)^2 + q(4)^2 2*(q(2)*q(3) + q(4)*q(1));
       2*(q(1)*q(3) + q(4)*q(2)) 2*(q(2)*q(3) - q(4)*q(1)) -q(1)^2 - q(2)^2 + q(3)^2 + q(4)^2];
    
    % % Rotation Rates inthe Body Frame w.r.t. Orbital Frame
    wo = w' - Aob*[0;-n;0];

    % % PD Gains
    % % First Approach
    % Kp=5; Kd=500; 
    % % Global variables used for tuning
    Kp = wp;  Kd = wd;
    % Tunned variables
    % Kp = 5;
    % Kd = 9;

    % % PD Controller for Reaction Wheels
    T = Kp*I*qe(1:3) + Kd*I*wo - cross(w',(I*w'+h'));
    
end