%% Run Each subscript at once not the whole script!!
close all; clear all; clc; 

x1_int = [0;-0.15;0.5];
x1Dot = [0;0;0];
x2_int = [0;0.15;0.5];
x2Dot = [0;0;0];
xc_int = [0;0;0.5];
xcDot = [0;0;0];

P.m = 0.755;
% P.m = 1.5;
P.g = 9.81;
P.Ix = 0.0820;
P.Iy = 0.0845;
P.Iz = 0.1377;
P.L = 0.225;
P.Mc = 1.5;
P.k = 2.5*10e4;
% P.k = 100;
P.C1Hat = eye(3); 
P.C2Hat = eye(3);
P.CcHat = eye(3);

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
P.Ts = 0.0005;
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

p0 = [x1_int;x2_int;xc_int];
q0 = [x1Dot;x2Dot;xcDot]; 
n=9;
m=3;
L=3;
NoiseVar = 0.00;
ThetaHat0= [ones(9,1)*1;1];
% ThetaHat0= [1; 0.5; 0.2; 2.0; 1.0; 0.5; 0.0; 0.3; 0.5; 3.0];
% ThetaHat0 = ones(10,1);
% ThetaHat0 = ThetaHat0+sqrt(NoiseVar)*randn(size(ThetaHat0));

% ThetaHat0= [1;0.5;1;1;0.5;1;0.5; 0.8; 1.0; 3.0];
%ThetaStar = ThetaHat0;
ThetaStar=[0.2061;0.2061;0.2061;0.2061;0.2061;0.2061;6.7315;2.6926;0;1.5];

T1 = 1;
T2 = 0.6;
Ts = P.Ts;
N = 150;
k1 = .1;
k2 = 0.001;
% kTheta = [k1*eye(6,6),zeros(6,4);zeros(4,6),k2*eye(4,4)];
kTheta = 0.5/N;
Gamma0 = 1*eye(numel(ThetaStar));
beta1 = 1;

sim('Force_Att_Controller_Separated')
%% Generates plot required for the Paper (Case-I) 

set(0,'defaulttextinterpreter','latex')
set(0,'defaultAxesTickLabelInterpreter','latex');
set(0,'defaultLegendInterpreter','latex');

fontname = 'cmss';
set(0,'defaultaxesfontname',fontname);
set(0,'defaulttextfontname',fontname);

fontsize = 16;
set(0,'defaultaxesfontsize',fontsize);
set(0,'defaulttextfontsize',fontsize);
% % 
figure
plot(t,xcDot(:,1),'k',t,xcDot(:,2),'m',t,xcDot(:,3),'--r','LineWidth',2.0)
xlabel('Time (sec)')
ylabel('Vel. of Payload (m/s)')
legend('X Velocity','Y Velocity','Z Velocity','Location','best');
set(gca,'FontSize',16)
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [7.5 2.5]);
set(gcf, 'PaperPosition', [0 0 7.5 2.5]);
set(gca,'LooseInset',get(gca,'TightInset'));
print -dpng  Plots_LCSS/VelLoadB.png 
hold on 
% % % % 
figure
plot(t,x1Dot(:,1),':r',t,xcDot(:,1),'--k',t,x1Dot(:,2),'m',t,xcDot(:,2),'-.b',t,x1Dot(:,3),':b',t,xcDot(:,3),'--k','LineWidth',2.0)
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
legend('Agent1,x','Payload,x','Agent1,y','Payload,y','Agent1,z','Payload,z','Location','NE');
set(gca,'FontSize',16)
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [7.5 2.5]);
set(gcf, 'PaperPosition', [0 0 7.5 2.5]);
set(gca,'LooseInset',get(gca,'TightInset'));
print -dpng Plots_LCSS/VelAgentsB.png ; 
% print -dpng Figures/VelAgents.png ; 
hold on

figure
plot(t,ThetaTildeNC(:,1),'--b',t,ThetaTildeNC(:,2),':r',t,ThetaTildeNC(:,3),'-.k','LineWidth',2.0);  
xlabel('Time (sec)')
ylabel('$\tilde{C}_1$')
legend('$\tilde{C}_1^x$','$\tilde C_1^y$','$\tilde C_1^z$','Location','SE') 
set(gca,'FontSize',16)
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [7.5 2.5]);
set(gcf, 'PaperPosition', [0 0 7.5 2.5]);
set(gca,'LooseInset',get(gca,'TightInset'));
print -dpng Plots_LCSS/C1TildeB.png ; 
% print -dpng ;  Figures/C1Tilde.png ; 
hold on 

figure
plot(t,ThetaTildeNC(:,7),'--b',t,ThetaTildeNC(:,8),':r',t,ThetaTildeNC(:,9),'-.k',t,ThetaTildeNC(:,10),'-.m','LineWidth',2.0);  
xlabel('Time (sec)')
ylabel('$\tilde{\theta}_c$')
legend('$\tilde{F}_d^x$','$\tilde{F}_d^x$','$\tilde{F}_d^z$','$\tilde{M}_c$') 
set(gca,'FontSize',16)
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [7.5 2.5]);
set(gcf, 'PaperPosition', [0 0 7.5 2.5]);
set(gca,'LooseInset',get(gca,'TightInset'));
print -dpng Plots_LCSS/ThetacTildeB.png ; 
% print Figures/ThetacTilde.png ; 
hold on 
% % % 
figure
plot(t,fdtilde(:,1),'--b',t,fdtilde(:,2),':r',t,fdtilde(:,3),'-.k','LineWidth',2.0);  
xlabel('Time (sec)')
ylabel('$\tilde{f}_1^d$')
legend('$\tilde{f}_{1,x}^d$','$\tilde{f}_{1,y}^d$','$\tilde{f}_{1,z}^d$','location', 'SE') 
set(gca,'FontSize',16)
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [7.5 2.5]);
set(gcf, 'PaperPosition', [0 0 7.5 2.5]);
set(gca,'LooseInset',get(gca,'TightInset'));
print -dpng Plots_LCSS/f1dTildeB.png ; 
% print Figures/f1dTilde.png ; 
hold on 
% 
figure
plot(x1(:,1),x1(:,2),':k',x2(:,1),x2(:,2),'-.b',xc(:,1),xc(:,2),'-.r','LineWidth',1.5)
xlabel('x position (m)')
ylabel('y postion (m)')
legend('Agent1','Agent','Payload','Location','best');
set(gca,'FontSize',16)
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [7.5 2.5]);
set(gcf, 'PaperPosition', [0 0 7.5 2.5]);
set(gca,'LooseInset',get(gca,'TightInset'));
print -dpng Plots_LCSS/XYPosB.png ; 
% print Figures/f1dTilde.png ; 
hold on 

figure
plot(t,CFc(:,1),'--b',t,CFc(:,2),':r',t,CFc(:,3),'r','LineWidth',1.5)
xlabel('time (sec)')
ylabel('Contact force (N)')
legend('$f_1^x$','$f_1^y$','$f_1^z$','Location','best','Orientation','horizontal','Interpreter','latex');
set(gca,'FontSize',16)
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [7.5 2.5]);
set(gcf, 'PaperPosition', [0 0 7.5 2.5]);
set(gca,'LooseInset',get(gca,'TightInset'));
print -dpng Plots_LCSS/ContactForceB.png ; 
hold on 

%% Plots for angles 
figure
plot(t,des_state1(:,1),'--r',t,state_quad1(:,1),':b',t,des_state2(:,1),':m',t,state_quad2(:,1),'-.k','LineWidth',2.0)
xlabel('Time (sec)','FontSize',16)
ylabel(' $\phi$ (rad)')
legend('$\phi_{1}^{des}$','$\phi_{1}$','$\phi_{2}^{des}$','$\phi_{2}$','Location','best');
set(gca,'FontSize',16);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [7.5 2.5]);
set(gcf, 'PaperPosition', [0 0 7.5 2.5]);
set(gca,'LooseInset',get(gca,'TightInset'));
print -dpng Plots_LCSS/RollAngleB.png ; 
hold on 
% 
figure
plot(t,des_state1(:,2),'--r',t,state_quad1(:,2),':b',t,des_state2(:,2),':m',t,state_quad2(:,2),'--k','LineWidth',2.0)
xlabel('Time (sec)')
ylabel('$\theta$ (rad)')
legend('$\theta_{1}^{des}$','$\theta_{1}$','$\theta_{2}^{des}$','$\theta_{2}$','Location','best','Interpreter','latex');
set(gca,'FontSize',16)
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [7.5 2.5]);
set(gcf, 'PaperPosition', [0 0 7.5 2.5]);
set(gca,'LooseInset',get(gca,'TightInset'));
print -dpng Plots_LCSS/PitchAngleB.png ; 
hold on 

%% Animation 
close all; 
clc; 
figure
n = length(q);
teta1 = linspace(0,2*pi,n/7500)';

% if Plots_LCSS/ConstantVelocityNewC.mp4
%     delete Plots_LCSS/ConstantVelocityNewC.mp4
% end 

v = VideoWriter('Plots_LCSS/TimeVarVelocity.mp4','MPEG-4');
open(v)

for i = 1:75:n % for regular sim use 1:100:n and use 1:10000:n for plot in the paper
    xb1 = x1(i,1);
    yb1 = x1(i,2);
    zb1 = x1(i,3);
    psi1  = state_quad1(i,1);      % z angle in the order ZYX [psi1 theta1 phi1] -->> [Zangle Yangle Xangle]
    theta1 = state_quad1(i,2);    %  y angle % Yaw - Pitch -Roll is not same everywhere, you can define based on your conventions
    phi1 = state_quad1(i,3);       % x angle  --->>ZYX in the order -->>YAW-PITCH-ROLL
    % Rotation matrix
    R_b1 = [ cos(psi1)*cos(theta1), cos(psi1)*sin(phi1)*sin(theta1) - cos(phi1)*sin(psi1), sin(phi1)*sin(psi1) + cos(phi1)*cos(psi1)*sin(theta1);
        cos(theta1)*sin(psi1), cos(phi1)*cos(psi1) + sin(phi1)*sin(psi1)*sin(theta1),   cos(phi1)*sin(psi1)*sin(theta1) - cos(psi1)*sin(phi1);
        -sin(theta1),         cos(theta1)*sin(phi1),                              cos(phi1)*cos(theta1)];
%     % Forward Kinematics
%     l1 = [0 -0.2 0.2];
    p_b1 = [xb1 yb1 zb1]';
 
    % Agent 2
    %Kinematics of right Aerial Manipulator (subscript 2)
    xb2 = x2(i,1);
    yb2 = x2(i,2);
    zb2 = x2(i,3);
    
    psi2 = state_quad2(i,1);     % z angle (Z)
    theta2 = state_quad2(i,2);   % y angle (Y)
    phi2 = state_quad2(i,3);      % x angle (x) -->> ZYX pair yaw-pitch roll pair oder is imporant!!!
   
    % Rotation matrix
    R_b2 = [ cos(psi2)*cos(theta2), cos(psi2)*sin(phi2)*sin(theta2) - cos(phi2)*sin(psi2), sin(phi2)*sin(psi2) + cos(phi2)*cos(psi2)*sin(theta2);
        cos(theta2)*sin(psi2), cos(phi2)*cos(psi2) + sin(phi2)*sin(psi2)*sin(theta2),   cos(phi2)*sin(psi2)*sin(theta2) - cos(psi2)*sin(phi2);
        -sin(theta2),         cos(theta2)*sin(phi2),                              cos(phi2)*cos(theta2)];
    
    % Forward Kinematics
%     l2 = [0 0.2 0.2]';
    p_b2 = [xb2 yb2 zb2]'; %postion of UAV with respect to inertial frame
  
    %Kinematics of 3rd Aerial Manipulator (subscript 2)
%     xb3 = q(i,13);
%     yb3 = q(i,14);
%     zb3 = q(i,15);
    
%     psi3 = state_quad3(i,1);     % z angle (Z)
%     theta3 = state_quad3(i,2);   % y angle (Y)
%     phi3 = state_quad3(i,3);      % x angle (x) -->> ZYX pair yaw-pitch roll pair oder is imporant!!!
%    
%     % Rotation matrix
%     R_b3 = [ cos(psi3)*cos(theta3), cos(psi3)*sin(phi3)*sin(theta3) - cos(phi3)*sin(psi3), sin(phi3)*sin(psi3) + cos(phi3)*cos(psi3)*sin(theta3);
%         cos(theta3)*sin(psi3), cos(phi3)*cos(psi3) + sin(phi3)*sin(psi3)*sin(theta3),   cos(phi3)*sin(psi3)*sin(theta3) - cos(psi3)*sin(phi3);
%         -sin(theta3),         cos(theta3)*sin(phi3),                              cos(phi3)*cos(theta3)];
    
% Forward Kinematics
%     l2 = [0 0.2 0.2]';
%     p_b3 = [xb3 yb3 zb3]'; %postion of UAV with respect to inertial frame
    
    % Star plotting % look plot3 in matlab
    plot3(p_b1(1,:),p_b1(2,:),p_b1(3,:),'*',...
    'LineWidth',1,...
    'MarkerSize',8,...
    'MarkerEdgeColor','w',...
    'MarkerFaceColor','w');
     hold on 
    plot3(p_b2(1,:),p_b2(2,:),p_b2(3,:),'o',...
    'LineWidth',1,...
    'MarkerSize',10,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor','r');
    hold on 
%     plot3(p_b3(1,:),p_b3(2,:),p_b3(3,:),'-o',...
%     'LineWidth',1,...
%     'MarkerSize',10,...
%     'MarkerEdgeColor','c',...
%     'MarkerFaceColor','y');
%      hold on 

        M1 = [0.15 0 0]'; % Motor 1 location in local frame
        Am1M1 = R_b1*M1 +  AM(i,1:3)'; % Motor 1 location in intertial frame Am1
        Am2M1 = R_b2*M1 +  AM(i,4:6)'; % Motor 1 location in interial frame Am2
%         Am3M1 = R_b3*M1 +  AM(i,7:9)';  % Motor 1 location in interial frame Am3
        
        M2 = [0 0.15 0]'; 
        Am1M2 = R_b1*M2 +  AM(i,1:3)';
        Am2M2 = R_b2*M2 +  AM(i,4:6)';
%         Am3M2 = R_b3*M2 +  AM(i,7:9)';  % Motor 2 location in interial frame Am3
        
        M3 = [-0.15 0 0]';
        Am1M3 = R_b1*M3 + AM(i,1:3)';
        Am2M3 = R_b2*M3 + AM(i,4:6)';
%         Am3M3 = R_b3*M3 + AM(i,7:9)';  % Motor 3 location in interial frame Am3
        
        M4 = [0 -0.15 0]';
        Am1M4 = R_b1*M4 + AM(i,1:3)';
        Am2M4 = R_b2*M4 +  AM(i,4:6)';
%         Am3M4 = R_b3*M4 +  AM(i,7:9)';  % Motor 4 location in interial frame Am3
            
        ext = [0 0 0.015]';  % extension for the motor 
        centerAm1M1 = Am1M1 + ext ; 
        centerAm1M2 = Am1M2 + ext;
        centerAm1M3 = Am1M3 + ext;
        centerAm1M4 = Am1M4 + ext;
        
        centerAm2M1 = Am2M1 + ext ;
        centerAm2M2 = Am2M2 + ext;
        centerAm2M3 = Am2M3 + ext;
        centerAm2M4 = Am2M4 + ext;
        
%         centerAm3M1 = Am3M1 + ext ;
%         centerAm3M2 = Am3M2 + ext;
%         centerAm3M3 = Am3M3 + ext;
%         centerAm3M4 = Am3M4 + ext;
        
        AM1 = AM(i,1:3)'; 
        AM2 = AM(i,4:6)';
%         AM3 = AM(i,7:9)';
        
        line([AM1(1,:),Am1M1(1,:)],[AM1(2,:),Am1M1(2,:)],[AM1(3,:),Am1M1(3,:)],'LineWidth',3.5,'Color','r')
        line([AM1(1,:),Am1M2(1,:)],[AM1(2,:),Am1M2(2,:)],[AM1(3,:),Am1M2(3,:)],'LineWidth',3.5,'Color','b')
        line([AM1(1,:),Am1M3(1,:)],[AM1(2,:),Am1M3(2,:)],[AM1(3,:),Am1M3(3,:)],'LineWidth',3.5,'Color','r')
        line([AM1(1,:),Am1M4(1,:)],[AM1(2,:),Am1M4(2,:)],[AM1(3,:),Am1M4(3,:)],'LineWidth',3.5,'Color','b')
        
        line([Am1M1(1,:), centerAm1M1(1,:)],[Am1M1(2,:),centerAm1M1(2,:)],[Am1M1(3,:),centerAm1M1(3,:)],'LineWidth',3.5,'Color','r')
        line([Am1M2(1,:), centerAm1M2(1,:)],[Am1M2(2,:),centerAm1M2(2,:)],[Am1M2(3,:),centerAm1M2(3,:)],'LineWidth',3.5,'Color','b')
        line([Am1M3(1,:), centerAm1M3(1,:)],[Am1M3(2,:),centerAm1M3(2,:)],[Am1M3(3,:),centerAm1M3(3,:)],'LineWidth',3.5,'Color','r')
        line([Am1M4(1,:), centerAm1M4(1,:)],[Am1M4(2,:),centerAm1M4(2,:)],[Am1M4(3,:),centerAm1M4(3,:)],'LineWidth',3.5,'Color','b')
        
        line([AM2(1,:),Am2M1(1,:)],[AM2(2,:),Am2M1(2,:)],[AM2(3,:),Am2M1(3,:)],'LineWidth',3.5,'Color','r')
        line([AM2(1,:),Am2M2(1,:)],[AM2(2,:),Am2M2(2,:)],[AM2(3,:),Am2M2(3,:)],'LineWidth',3.5,'Color','b')
        line([AM2(1,:),Am2M3(1,:)],[AM2(2,:),Am2M3(2,:)],[AM2(3,:),Am2M3(3,:)],'LineWidth',3.5,'Color','r')
        line([AM2(1,:),Am2M4(1,:)],[AM2(2,:),Am2M4(2,:)],[AM2(3,:),Am2M4(3,:)],'LineWidth',3.5,'Color','b')
        
        line([Am2M1(1,:), centerAm2M1(1,:)],[Am2M1(2,:),centerAm2M1(2,:)],[Am2M1(3,:),centerAm2M1(3,:)],'LineWidth',3.5,'Color','r')
        line([Am2M2(1,:), centerAm2M2(1,:)],[Am2M2(2,:),centerAm2M2(2,:)],[Am2M2(3,:),centerAm2M2(3,:)],'LineWidth',3.5,'Color','b')
        line([Am2M3(1,:), centerAm2M3(1,:)],[Am2M3(2,:),centerAm2M3(2,:)],[Am2M3(3,:),centerAm2M3(3,:)],'LineWidth',3.5,'Color','r')
        line([Am2M4(1,:), centerAm2M4(1,:)],[Am2M4(2,:),centerAm2M4(2,:)],[Am2M4(3,:),centerAm2M4(3,:)],'LineWidth',3.5,'Color','b')
%         
%         line([AM3(1,:),Am3M1(1,:)],[AM3(2,:),Am3M1(2,:)],[AM3(3,:),Am3M1(3,:)],'LineWidth',3.5,'Color','r')
%         line([AM3(1,:),Am3M2(1,:)],[AM3(2,:),Am3M2(2,:)],[AM3(3,:),Am3M2(3,:)],'LineWidth',3.5,'Color','b')
%         line([AM3(1,:),Am3M3(1,:)],[AM3(2,:),Am3M3(2,:)],[AM3(3,:),Am3M3(3,:)],'LineWidth',3.5,'Color','r')
%         line([AM3(1,:),Am3M4(1,:)],[AM3(2,:),Am3M4(2,:)],[AM3(3,:),Am3M4(3,:)],'LineWidth',3.5,'Color','b')
%         
%         line([Am3M1(1,:), centerAm3M1(1,:)],[Am3M1(2,:),centerAm3M1(2,:)],[Am3M1(3,:),centerAm3M1(3,:)],'LineWidth',3.5,'Color','r')
%         line([Am3M2(1,:), centerAm3M2(1,:)],[Am3M2(2,:),centerAm3M2(2,:)],[Am3M2(3,:),centerAm3M2(3,:)],'LineWidth',3.5,'Color','b')
%         line([Am3M3(1,:), centerAm3M3(1,:)],[Am3M3(2,:),centerAm3M3(2,:)],[Am3M3(3,:),centerAm3M3(3,:)],'LineWidth',3.5,'Color','r')
%         line([Am3M4(1,:), centerAm3M4(1,:)],[Am3M4(2,:),centerAm3M4(2,:)],[Am3M4(3,:),centerAm3M4(3,:)],'LineWidth',3.5,'Color','b')
%         
        hold on
        line([AM1(1,:),p_b1(1,:)],[AM1(2,:),p_b1(2,:)],[AM1(3,:),p_b1(3,:)],'LineWidth',2.0,'Color','r')
        line([AM2(1,:),p_b2(1,:)],[AM2(2,:),p_b2(2,:)],[AM2(3,:),p_b2(3,:)],'LineWidth',2.0,'Color','m')
%         line([AM3(1,:),p_b3(1,:)],[AM3(2,:),p_b3(2,:)],[AM3(3,:),p_b3(3,:)],'LineWidth',2.0,'Color','b')
        hold on
        
        % Draw the extension for line
        r = .075; %d = 0.125; h = .025; %inches: rotor dia., quad motor distance from
        % cm, and rotor height above arms (entirely cosmetic)
        
        % Construct rotor representations
        %     N = [d  0 h].';% m1 rotor center
        %     E = [0 -d h].';% m4 rotor center
        %     W = [0  d h].';% m2 rotor center
        %     S = [-d 0 h].';% m3 rotor center
        N1 = centerAm1M1;
        E1 = centerAm1M2;
        W1 = centerAm1M3;
        S1 = centerAm1M4;
        
        N2 = centerAm2M1;
        E2 = centerAm2M2;
        W2 = centerAm2M3;
        S2 = centerAm2M4;
        
%         N3 = centerAm3M1;
%         E3 = centerAm3M2;
%         W3 = centerAm3M3;
%         S3 = centerAm3M4;
        
        Nr = circlePoints(zeros(3,1), r, 201); % Nr = [Nr Nr(:,1)]; % Rotor blade circles
        Er = circlePoints(zeros(3,1), r, 201); % Er = [Er Er(:,1)];
        Wr = circlePoints(zeros(3,1), r, 201); % Wr = [Wr Wr(:,1)];
        Sr = circlePoints(zeros(3,1), r, 201); % Sr = [Sr Sr(:,1)];
        % Motors connecting to center of blade circles
        
        % Create 3d fills
        %     Nr=Rz*Nr;
        %     Er=Rz*Er;
        %     Wr=Rz*Wr;
        %     %
        %     Rz = [ sqrt(2)/2, sqrt(2)/2, 0;
        %                -sqrt(2)/2,sqrt(2)/2, 0;
        %                        0,          0, 1];
        
        
        %  R_b1 = [ cos(0)*cos(theta1), cos(0)*sin(phi1)*sin(theta1) - cos(phi1)*sin(0), sin(phi1)*sin(0) + cos(phi1)*cos(0)*sin(theta1);
        %         cos(theta1)*sin(0), cos(phi1)*cos(0) + sin(phi1)*sin(0)*sin(theta1),   cos(phi1)*sin(0)*sin(theta1) - cos(0)*sin(phi1);
        %         -sin(theta1),         cos(theta1)*sin(phi1),                              cos(phi1)*cos(theta1)];
        %     p_b = [0 -0.3 0]';
        
        NrR1 = R_b1*Nr + N1;
        ErR1 = R_b1*Er + E1;
        WrR1 = R_b1*Wr + W1;
        SrR1 = R_b1*Sr + S1;
        
        NrR2 = R_b2*Nr + N2;
        ErR2 = R_b2*Er + E2;
        WrR2 = R_b2*Wr + W2;
        SrR2 = R_b2*Sr + S2;
        
%         NrR3 = R_b3*Nr + N3;
%         ErR3 = R_b3*Er + E3;
%         WrR3 = R_b3*Wr + W3;
%         SrR3 = R_b3*Sr + S3;
        
        plot3(NrR1(1,:),NrR1(2,:),NrR1(3,:),'LineWidth',1.5,'Color','g')
        plot3(ErR1(1,:),ErR1(2,:),ErR1(3,:),'LineWidth',1.5,'Color','r')
        plot3(WrR1(1,:),WrR1(2,:),WrR1(3,:),'LineWidth',1.5,'Color','g')
        plot3(SrR1(1,:),SrR1(2,:),SrR1(3,:),'LineWidth',1.5,'Color','r')
        hold on
        plot3(NrR2(1,:),NrR2(2,:),NrR2(3,:),'LineWidth',1.5,'Color','g')
        plot3(ErR2(1,:),ErR2(2,:),ErR2(3,:),'LineWidth',1.5,'Color','r')
        plot3(WrR2(1,:),WrR2(2,:),WrR2(3,:),'LineWidth',1.5,'Color','g')
        plot3(SrR2(1,:),SrR2(2,:),SrR2(3,:),'LineWidth',1.5,'Color','r')
        hold on 
%         plot3(NrR3(1,:),NrR3(2,:),NrR3(3,:),'LineWidth',1.5,'Color','g')
%         plot3(ErR3(1,:),ErR3(2,:),ErR3(3,:),'LineWidth',1.5,'Color','r')
%         plot3(WrR3(1,:),WrR3(2,:),WrR3(3,:),'LineWidth',1.5,'Color','g')
%         plot3(SrR3(1,:),SrR3(2,:),SrR3(3,:),'LineWidth',1.5,'Color','r')
%         hold on 

        a = 0.135; b = a;  c = 0.02;
        
        Top = [ a/2,   0,-a/2,   0;
            0, b/2,   0,-b/2;
            c/2, c/2, c/2, c/2];
        Bot = vertcat(Top(1:2,:),-Top(3,:));
        NEB = [ a/2, a/2,   0,   0;
            0,   0, b/2, b/2;
            c/2,-c/2,-c/2, c/2];
        NWB = [ a/2, a/2,   0,   0;
            0,   0,-b/2,-b/2;
            c/2,-c/2,-c/2, c/2];
        SEB = -NWB;
        SWB = -NEB;
        
%         OriginAm1 = [p_b1, p_b1 , p_b1 , p_b1];
%         OriginAm2 = [p_b2, p_b2, p_b2, p_b2];
        OriginAm1 = [AM(i,1:3)', AM(i,1:3)' , AM(i,1:3)' , AM(i,1:3)'];
        OriginAm2 = [AM(i,4:6)', AM(i,4:6)' , AM(i,4:6)' , AM(i,4:6)'];
%         OriginAm3 = [AM(i,7:9)', AM(i,7:9)' , AM(i,7:9)' , AM(i,7:9)'];

        TopR1 = OriginAm1  + R_b1*Top;
        BotR1 = OriginAm1 + R_b1*Bot;
        NEBR1 = OriginAm1 + R_b1*NEB;
        NWBR1 = OriginAm1 + R_b1*NWB;
        SWBR1 = OriginAm1 + R_b1*SWB;
        SEBR1 = OriginAm1  + R_b1*SEB;
        
        TopR2 = OriginAm2 + R_b2*Top;
        BotR2 = OriginAm2 + R_b2*Bot;
        NEBR2 = OriginAm2 + R_b2*NEB;
        NWBR2 = OriginAm2 + R_b2*NWB;
        SWBR2 = OriginAm2 + R_b2*SWB;
        SEBR2 = OriginAm2 + R_b2*SEB;
        
%         TopR3 = OriginAm3 + R_b3*Top;
%         BotR3 = OriginAm3 + R_b3*Bot;
%         NEBR3 = OriginAm3 + R_b3*NEB;
%         NWBR3 = OriginAm3 + R_b3*NWB;
%         SWBR3 = OriginAm3 + R_b3*SWB;
%         SEBR3 = OriginAm3 + R_b3*SEB;
        
        fill3(TopR1(1,:),TopR1(2,:),TopR1(3,:),'r'); alpha(0.8);
        fill3(BotR1(1,:),BotR1(2,:),BotR1(3,:),'r'); alpha(0.8);
        
        grey = [0.5 0.5 0.5];
        ne  = fill3(NEBR1(1,:),NEBR1(2,:),NEBR1(3,:),grey); alpha(ne,0.8); % North East surface
        nw  = fill3(NWBR1(1,:),NWBR1(2,:),NWBR1(3,:),grey); alpha(nw,0.8); % North West surface
        sw  = fill3(SWBR1(1,:),SWBR1(2,:),SWBR1(3,:),grey); alpha(sw,0.8); % South West surface
        se  = fill3(SEBR1(1,:),SEBR1(2,:),SEBR1(3,:),grey); alpha(se,0.8); % South East surface
       
        az = 71; 
        e1 = 45;
%         az = 84;
%         e1 = 25;
        view([az,e1]);
        hold on
        
        fill3(TopR2(1,:),TopR2(2,:),TopR2(3,:),'r'); alpha(0.8);
        fill3(BotR2(1,:),BotR2(2,:),BotR2(3,:),'r'); alpha(0.8);
        grey = [0.5 0.5 0.5];
        ne  = fill3(NEBR2(1,:),NEBR2(2,:),NEBR2(3,:),grey); alpha(ne,0.8); % North East surface
        nw  = fill3(NWBR2(1,:),NWBR2(2,:),NWBR2(3,:),grey); alpha(nw,0.8); % North West surface
        sw  = fill3(SWBR2(1,:),SWBR2(2,:),SWBR2(3,:),grey); alpha(sw,0.8); % South West surface
        se  = fill3(SEBR2(1,:),SEBR2(2,:),SEBR2(3,:),grey); alpha(se,0.8); % South East surface
        
        hold on 
        
%         fill3(TopR3(1,:),TopR3(2,:),TopR3(3,:),'r'); alpha(0.8);
%         fill3(BotR3(1,:),BotR3(2,:),BotR3(3,:),'r'); alpha(0.8);
%         grey = [0.5 0.5 0.5];
%         ne  = fill3(NEBR3(1,:),NEBR3(2,:),NEBR3(3,:),grey); alpha(ne,0.8); % North East surface
%         nw  = fill3(NWBR3(1,:),NWBR3(2,:),NWBR3(3,:),grey); alpha(nw,0.8); % North West surface
%         sw  = fill3(SWBR3(1,:),SWBR3(2,:),SWBR3(3,:),grey); alpha(sw,0.8); % South West surface
%         se  = fill3(SEBR3(1,:),SEBR3(2,:),SEBR3(3,:),grey); alpha(se,0.8); % South East surface
%         
        hold on
       
        [xx,yy,zz] = sphere;
        rr = 0.15;
        surface(xx*rr+p(i,7),yy*rr+p(i,8),zz*rr+(p(i,9))) % , C, ...
%             'FaceColor','texturemap',...
%             'EdgeColor','none',...
%             'CDataMapping','direct')
%         colormap(map)
%         az = 72;
%         el = 54;
        view(az, e1);
        hold on 
        
        h1 = plot3(AM(1:i,1),AM(1:i,2),AM(1:i,3),'k',AM(1:i,4),AM(1:i,5),AM(1:i,6),'b', xc(1:i,1),xc(1:i,2),xc(1:i,3),'r','LineWidth',1.0);
        legend([h1],{'Agent1','Agent2', 'Load'},'location', 'northeast')

%         axis ([-1.0 30 -5 20 -0.5 1.2])
        axis ([-1.0 18.2 -0.5 7.8 -0.2 0.8])

%         xlim([-0.5 20])
%         ylim([-1.5 1.5])
%         zlim([0.0 1.2])
%         axis equal
%         axis ([-0.4 6.0 -0.55 0.55 -0.5 1.0]) % Circle 
%          axis ([-0.5 5.3 -0.45 5.3 0.4 0.75]) % var vd NAASS paper
        %
        % axis ([-0.4 5.2 -2.4 2.4 0.2 2.0])
            xlabel(' x axis')
            zlabel('z axis')
            ylabel('y axis')
        drawnow update
        grid on
        %     axis tight manual
        M(i) = getframe(gcf);
%         im(:,:,:,i) = frame2im(M(i));
%         getframe(gcf,),
        set(gca,'nextplot','replacechildren');
        title(sprintf('Time: %0.2f sec', t(i)));
        
        view(az, e1);
       writeVideo(v,M(i))
% hold on % Turn on for 3D position
end 
hold on % Turn this on for video 
        
close(v)