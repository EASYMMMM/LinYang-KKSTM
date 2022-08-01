% clear;
% clc;

robot = VrepConnector(19999,0.05);

addpath('C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client_rawing\EMG_IMU')
addpath 'C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main'
addpath 'C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client'

            CHANGE=0;
            now_pos_3=[0.1 -0.45 0.2]';
            [lastq,lastR,total_act way_points which_state_now myspace]= RL2m3m3_maze(now_pos_3,0,CHANGE,0,0,0,0);
            all0=[];
            for ii = 1:size(lastq,2)
                if all(lastq(:,ii)==0)
                    all0=[all0 ii];
                end
            end
% way_points=now
i=1;
init_theta=0;
xd=way_points(1,i)
yd=way_points(2,i)
zd=way_points(3,i)
    init_theta1=180;
    init_theta2=0;
    init_theta3=0;
    [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
All_theta=All_theta*pi/180
what=All_theta(:,2)
hh=directKinematics(what)
hhh=ForwardKinematics(what,1)
robot.ApplyPosi2(what);


