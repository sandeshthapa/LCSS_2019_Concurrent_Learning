%P = struct;
% Modified by Sandesh Thapa
% Writen by Sam Alison 

%dynamics params
P.m = 0.755;
P.g = 9.81;
P.Ix = 0.0820;
P.Iy = 0.0845;
P.Iz = 0.1377;
P.L = 0.225;
P.Mc = 1.0;
P.k = 2.5*10e4;
P.C1Hat = eye(3); 
P.C2Hat = eye(3);
P.CcHat = eye(3);


%%

% P.m = 1.5*4;
% P.g = 9.81;
% P.Ix = 0.0347563*4;
% P.Iy = 0.0458929*4;
% P.Iz = 0.0977*4;
% P.L = 0.47/2;

P.Cd1 = 0.2;%4.842/(200);
P.Cd2 = 0.2;%4.842/(200);
P.Cd3 = 0.2;%4.842/(200);

%attitude controller params
P.K2 = -14.6211*1.5;
P.K3 = -14.6211*1.5;
P.K4 = -32*1.5;

P.Kp2 = 1.8668*5/2;
P.Kp3 = 1.8572*5/2;
P.Kp4 = 1.5064*5/2;

P.Kd2 = .2340*1*0.8;
P.Kd3 = .2334*1*0.8;
P.Kd4 = .1870*1*0.8;


%position controller params
P.Ts = 0.001;
P.kp = 0.3;
P.kd = 0.25;
%integral term needs to be scaled by timestep
P.ki = 2E-4*(1000*P.Ts);

%altitude
P.k2 = 1.5;
P.kd2 = .05;
P.ki2 = 3E-4*(1000*P.Ts);

%Velocity controller params
P.kp_v = 0.5;
P.kd_v = 0.6;
P.ki_v = 0.0005;

%From Dr. Bai's model
% wind parameters
%50 meter alt
P.L_wx = 200;
P.L_wy = 200;
P.L_wz = 50;
%100 meter alt
% P.L_wx = 263;
% P.L_wy = 263;
% P.L_wz = 100;
% %150 meter alt
% P.L_wx = 287;
% P.L_wy = 287;
% P.L_wz = 150;

%P.sigma_wx = 1; 
%P.sigma_wy = 1;
%P.sigma_wz = 0.7;

%P.Va0 = 5;