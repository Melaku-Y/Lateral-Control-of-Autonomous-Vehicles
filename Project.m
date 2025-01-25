%% --- VEHICLE PARAMETERS ---
m   = 1500;   % Vehicle mass (kg)
Iz  = 2734;   % Yaw moment of inertia (kg·m^2)
Lf  = 1.3;    % CG-to-front-axle (m)
Lr  = 1.4;    % CG-to-rear-axle  (m)
Cf  = 30000;  % Front cornering stiffness (N/rad)
Cr  = 40000;  % Rear cornering stiffness (N/rad)

%% --- GLOBALS FOR SIMULINK MODEL ---
Ts = 0.01;    % sample time (s)
vx = 20;      % forward velocity (m/s)
Vy = 0;       % lateral velocity (if used inside the model)

% Put them into the base workspace so the Simulink model sees them:
assignin('base','m',m);
assignin('base','Iz',Iz);
assignin('base','Lf',Lf);
assignin('base','Lr',Lr);
assignin('base','Cf',Cf);
assignin('base','Cr',Cr);
assignin('base','Ts',Ts);
assignin('base','vx',vx);
assignin('base','Vy',Vy);

%% --- LQR DESIGN ---
% Continuous-time A, B at speed vx:
A_ct = [ -(Cf+Cr)/(m*vx),  -vx - (Cf*Lf - Cr*Lr)/(m*vx),  0;
         -(Cf*Lf - Cr*Lr)/(Iz*vx), -(Cf*Lf^2 + Cr*Lr^2)/(Iz*vx), 0;
         0, 1, 0 ];

B_ct = [  Cf/m;
         (Cf*Lf)/Iz;
          0 ];

C_ct = eye(3);
D_ct = [0;0;0];

% Discretize:
sys_ct = ss(A_ct, B_ct, C_ct, D_ct);
sys_d  = c2d(sys_ct, Ts);
ad     = sys_d.A;
bd     = sys_d.B;
cd     = sys_d.C;

% Weights:
Q_lqr = diag([0.1,  20,  0.001]);%[0.1,  20,  0.001 ]
R_lqr = 2;
K_lqr = dlqr(ad, bd, Q_lqr, R_lqr);

% Store for Simulink
assignin('base','K_lqr',K_lqr);

%% --- LPV (POLYTOPIC) DESIGN VIA CVX ---
% Parameter range for vx:
minval = 5;
maxval = 20;

rho1min = minval;   rho1max = maxval;
rho2min = 1/maxval; rho2max = 1/minval;

%% Four corner A-matrices (continuous):
A1_ct = [ -(Cf+Cr)*rho2min/m, -rho1min + (Cr*Lr - Cf*Lf)*rho2min/m, 0;
          (-Lf*Cf + Lr*Cr)*rho2min/Iz, -(Lf^2*Cf + Lr^2*Cr)*rho2min/Iz, 0;
           0, 1, 0 ];

A2_ct = [ -(Cf+Cr)*rho2max/m, -rho1min + (Cr*Lr - Cf*Lf)*rho2max/m, 0;
          (-Lf*Cf + Lr*Cr)*rho2max/Iz, -(Lf^2*Cf + Lr^2*Cr)*rho2max/Iz, 0;
           0, 1, 0 ];

A3_ct = [ -(Cf+Cr)*rho2min/m, -rho1max + (Cr*Lr - Cf*Lf)*rho2min/m, 0;
          (-Lf*Cf + Lr*Cr)*rho2min/Iz, -(Lf^2*Cf + Lr^2*Cr)*rho2min/Iz, 0;
           0, 1, 0 ];

A4_ct = [ -(Cf+Cr)*rho2max/m, -rho1max + (Cr*Lr - Cf*Lf)*rho2max/m, 0;
          (-Lf*Cf + Lr*Cr)*rho2max/Iz, -(Lf^2*Cf + Lr^2*Cr)*rho2max/Iz, 0;
           0, 1, 0 ];

B_ct_lpv = [ Cf/m; Cf*Lf/Iz; 0 ];

% Discretize corners:
Add1 = eye(3) + Ts*A1_ct;
Add2 = eye(3) + Ts*A2_ct;
Add3 = eye(3) + Ts*A3_ct;
Ad4 = eye(3) + Ts*A4_ct;
Bdd  = Ts * B_ct_lpv;

nx = 3; nu = 1;

% LPV weighting
Q_lpv = diag([300, 10, 0.1]);
R_lpv = 1;

% LMI data:
C_lmi = [ sqrt(Q_lpv);
          zeros(nu, nx) ];
D_lmi = [ zeros(nx, nu);
          sqrt(R_lpv)*eye(nu) ];
clc
% Solve polytopic LMI with CVX:
cvx_clear
cvx_begin sdp
    variable P(nx,nx) symmetric
    variable Y1(nu,nx)
    variable Y2(nu,nx)
    variable Y3(nu,nx)
    variable Y4(nu,nx)
    variable W(nx+nu,nx+nu) symmetric

    minimize( trace(W) )
    subject to
        P >= 0;

        % corner #1
        [ -P+Q_lpv, Add1*P - Bdd*Y1;
          (Add1*P - Bdd*Y1)', -P ] <= 0;

        [ W, C_lmi*P + D_lmi*Y1;
          (C_lmi*P + D_lmi*Y1)', P ] >= 0;

        % corner #2
        [ -P+Q_lpv, Add2*P - Bdd*Y2;
          (Add2*P - Bdd*Y2)', -P ] <= 0;

        [ W, C_lmi*P + D_lmi*Y2;
          (C_lmi*P + D_lmi*Y2)', P ] >= 0;

        % corner #3
        [ -P+Q_lpv, Add3*P - Bdd*Y3;
          (Add3*P - Bdd*Y3)', -P ] <= 0;

        [ W, C_lmi*P + D_lmi*Y3;
          (C_lmi*P + D_lmi*Y3)', P ] >= 0;

        % corner #4
        [ -P+Q_lpv, Ad4*P - Bdd*Y4;
          (Ad4*P - Bdd*Y4)', -P ] <= 0;

        [ W, C_lmi*P + D_lmi*Y4;
          (C_lmi*P + D_lmi*Y4)', P ] >= 0;
cvx_end

% Gains
K1 = Y1*inv(P);
K2 = Y2*inv(P);
K3 = Y3*inv(P);
K4 = Y4*inv(P);

% Put them in base workspace:
assignin('base','K1',K1);
assignin('base','K2',K2);
assignin('base','K3',K3);
assignin('base','K4',K4);

%% Three corner A-matrices (continuous):
A1_c = [ -(Cf+Cr)*rho2min/m, -rho1min + (Cr*Lr - Cf*Lf)*rho2min/m, 0;
          (-Lf*Cf + Lr*Cr)*rho2min/Iz, -(Lf^2*Cf + Lr^2*Cr)*rho2min/Iz, 0;
           0, 1, 0 ];

A2_c = [ -(Cf+Cr)*rho2max/m, -rho1min + (Cr*Lr - Cf*Lf)*rho2max/m, 0;
          (-Lf*Cf + Lr*Cr)*rho2max/Iz, -(Lf^2*Cf + Lr^2*Cr)*rho2max/Iz, 0;
           0, 1, 0 ];

A3_c = [ -(Cf+Cr)*rho2min/m, -rho1max + (Cr*Lr - Cf*Lf)*rho2min/m, 0;
          (-Lf*Cf + Lr*Cr)*rho2min/Iz, -(Lf^2*Cf + Lr^2*Cr)*rho2min/Iz, 0;
           0, 1, 0 ];

B_c_lpv = [ Cf/m; Cf*Lf/Iz; 0 ];

% Discretize corners:
Add1 = eye(3) + Ts*A1_c;
Add2 = eye(3) + Ts*A2_c;
Add3 = eye(3) + Ts*A3_c;
Bdd  = Ts * B_c_lpv;

nx = 3; nu = 1;

% LPV weighting
Q_lpv = diag([300, 10, 0.1]);
R_lpv = 1;

% LMI data:
C_lmi = [ sqrt(Q_lpv);
          zeros(nu, nx) ];
D_lmi = [ zeros(nx, nu);
          sqrt(R_lpv)*eye(nu) ];
clc
% Solve polytopic LMI with CVX:
cvx_clear
cvx_begin sdp
    variable P(nx,nx) symmetric
    variable Y1(nu,nx)
    variable Y2(nu,nx)
    variable Y3(nu,nx)
    variable W(nx+nu,nx+nu) symmetric

    minimize( trace(W) )
    subject to
        P >= 0;

        % corner #1
        [ -P+Q_lpv, Add1*P - Bdd*Y1;
          (Add1*P - Bdd*Y1)', -P ] <= 0;

        [ W, C_lmi*P + D_lmi*Y1;
          (C_lmi*P + D_lmi*Y1)', P ] >= 0;

        % corner #2
        [ -P+Q_lpv, Add2*P - Bdd*Y2;
          (Add2*P - Bdd*Y2)', -P ] <= 0;

        [ W, C_lmi*P + D_lmi*Y2;
          (C_lmi*P + D_lmi*Y2)', P ] >= 0;

        % corner #3
        [ -P+Q_lpv, Add3*P - Bdd*Y3;
          (Add3*P - Bdd*Y3)', -P ] <= 0;

        [ W, C_lmi*P + D_lmi*Y3;
          (C_lmi*P + D_lmi*Y3)', P ] >= 0;
cvx_end

% Gains
Kk1 = Y1*inv(P);
Kk2 = Y2*inv(P);
Kk3 = Y3*inv(P);

% Put them in base workspace:
assignin('base','Kk1',Kk1);
assignin('base','Kk2',Kk2);
assignin('base','Kk3',Kk3);
%% --- ADDITIONAL MATRICES (A_T, B_T, etc.) FOR SIMULINK BLOCKS ---
A_T = [ -(Cf+Cr)/(m*vx),    -vx + (Cr*Lr - Cf*Lf)/(m*vx),  0;
        (-Lf*Cf + Lr*Cr)/(Iz*vx), -(Lf^2*Cf + Lr^2*Cr)/(Iz*vx), 0;
         0, 1, 0 ];

B_T = [ Cf/m; Cf*Lf/Iz; 0 ];

C_T = diag([1, 1, 1]);
D_T = [0;0;0];

AT_D = eye(size(A_T)) + Ts*A_T;
BT_D = Ts * B_T;

assignin('base','A_T',A_T);
assignin('base','B_T',B_T);
assignin('base','C_T',C_T);
assignin('base','D_T',D_T);
assignin('base','AT_D',AT_D);
assignin('base','BT_D',BT_D);




%% open_system('simulator_lqr_lpv.slx');

% -- Run the simulation, capturing the workspace outputs in 'simOut' --
simOut = sim('simulator_lqr_lpv.slx', ...
             'ReturnWorkspaceOutputs','on');

% This returns a struct containing all data from 'To Workspace' blocks.

% -- Extract arrays from simOut --
% The 'simOut' struct has fields named exactly as the "Variable name"
% in each To Workspace block.
REFERENCE    = simOut.get('REFERENCE');     % Nx2 array
lQROUTXY     = simOut.get('lQROUTXY');      % Nx2 array
LPV_VARYING_3  = simOut.get('LPV_VARYING_3');   % Nx2 array
LPV_VARYING_4  = simOut.get('LPV_VARYING_4');   % Nx2 array

% Now index the arrays:
X_ref = REFERENCE(:,1);
Y_ref = REFERENCE(:,2);

x_lqr = lQROUTXY(:,1);
y_lqr = lQROUTXY(:,2);

x_lpv_3 = LPV_VARYING_3(:,1);
y_lpv_3 = LPV_VARYING_3(:,2);

x_lpv_4 = LPV_VARYING_4(:,1);
y_lpv_4 = LPV_VARYING_4(:,2);

% -- Plot all on one figure --
figure; hold on; grid on;
plot(X_ref, Y_ref, '-r','LineWidth',1);
plot(x_lqr, y_lqr, '-b','LineWidth',1);
plot(x_lpv_3, y_lpv_3, '-g','LineWidth',1);
plot(x_lpv_4, y_lpv_4, '-k','LineWidth',1);
xlabel('X'); ylabel('Y');
legend('Reference','LQR','LPV3corner','LPV4corner','Location','Best');
title('Comparison: Reference vs. LQR vs. LPV ');
