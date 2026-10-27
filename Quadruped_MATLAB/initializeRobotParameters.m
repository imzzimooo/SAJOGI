%% Quadruped robot parameters

% Copyright 2019-2022 The MathWorks Inc.

% Body and leg geometry
L = 0.226;        % Distance b/w front and rear hip joints (m)
L_back = 0.305; % Length of the torso (m)
l1 = 0.1422;  % Link #1 length (m)
l2 = 0.115;  % Link #2 length (m)
torso_dim=[L_back 0.04875 0.152];
%torso COM
torso_COM=[-0.165 -1.237 -1.096]*0.001;
%link1 COM
link1_COM=[0 -43.026 0]*0.001;
link2_COM=[0 -5.8413 0]*0.001;
% Robot mass
M = 1.318;      % Torso mass (kg)
mU=1; %upper torso mass(unknown)
mL=1; %lower torso mass(unknown)
m1 = 0.17824;   % Leg link #1 mass (kg)
m2 = 0.03558;   % Leg link #2 mass (kg)

% Moment of inertia (kg-m^2)
torso_MOI = [0.01284 0.003082 0.01451];
torso_POI = [-10467.0584388 16160.2925201 4299.4783727] * 1e-9;
% Gravity (m/s^2)
g = -9.81;

% Sample Time (s)
Ts = 0.025;
%gait period
period=0.8;
%rotation angle of each pulse
dTheta=20*pi/180;
%pulse duration
Tp=0.15;
% Simulation Time (s)
Tf = 10;

% Desired height of the torso (m)
h_final = 0.2;

% Initial body height and foot displacement (m)
init_foot_disp_x = 0;
init_body_height = h_final;

% Initial joint angles (rad) and velocities (rad/s)
d2r = pi/180;
init_ang_FL = d2r * quadrupedInverseKinematics(init_foot_disp_x,-init_body_height,l1,l2);
init_ang_FR = init_ang_FL;
init_ang_RL = init_ang_FL;
init_ang_RR = init_ang_FL;
init_whip_FL = 0;
init_whip_FR = 0;
init_whip_RL = 0;
init_whip_RR = 0;

% initial height (m)
foot_height = 0.05*l2*(1-sin(2*pi-(3*pi/2+init_ang_FL(1)+init_ang_FL(2))));
y_init = init_body_height + foot_height;

% Initial body speeds in x,y (m/s)
vx_init = 0;
vy_init = 0;

% Contact friction properties
mu_kinetic = 0.88;
mu_static = 0.9;
v_thres = 0.001;

% Ground properties
ground.stiffness = 2e3;
ground.damping = 1e2;
ground.length = 100;
ground.width = 1;
ground.depth = 0.01;

% Hip and Knee Joint properties
joint.stiffness = 0;
joint.damping = 8;
joint.limitStiffness = 500;
joint.limitDamping = 50;
joint.transitionWidth = 2 * d2r;
hip_eq_angle = 0;
knee_eq_angle = 0;

% Define limits on variables
u_max = 10;                        % max joint torque = +/- u_max
y_min = 0.5;                       % min height of body from ground
z_max = 0.5;                       % max translation in z
vx_max = 2.5;                      % max horizontal speed of body
vy_max = 2.5;                      % max vertical speed of body
vz_max = 2.5;                      % max lateral speed of body
roll_max = 10 * d2r;               % max roll angle of body
pitch_max = 10 * d2r;              % max pitch angle of body
yaw_max = 20 * d2r;                % max yaw angle of body
omega_x_max = pi/2;                % max angular speed about x
omega_y_max = pi/2;                % max angular speed about x
omega_z_max = pi/2;                % max angular speed about x
q_hip_min = -120 * d2r;            % hip and knee joint angle limit
q_hip_max = -30 * d2r;
q_knee_min = 60 * d2r;
q_knee_max = 140 * d2r;
w_max = 2*pi*60/60;                % hip and knee joint angular speed limit
y_max = l1*cos(q_hip_max) + l2*cos(q_hip_max+q_knee_min);  % max height of body from ground
normal_force_max = ((M+4*m1+4*m2)*abs(g))/4;
friction_force_max = mu_static * normal_force_max;


