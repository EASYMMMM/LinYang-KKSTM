clear;
clc;close all;
% I prefer to use this file to test my trajotory in 0428
%% Parameters
% Controller gain
Kp_joint = eye(7)*20;
Kp_cart = eye(3)*15;
% Redundancy gain
k0 = 10; %0 for standard (non collision-free) motion planning 
% Integration Time Step
Ts = 0.05;
% Time execution
T_tot = 20;
% Controller type
controller = "joint";
%% Simulation
robot = VrepConnector(19999,0.05);
addpath('C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client_rawing\EMG_IMU')
addpath 'C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main'
addpath 'C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client'

            CHANGE=0;
            now_pos_3=[0.1 -0.45 0.2]';
            [lastq,lastR,total_act way_points which_state_now myspace cartis_obs]= RL2m3m3_maze(now_pos_3,0,CHANGE,0,0,0,0);
            
            all0=[];
            for ii = 1:size(lastq,2)
                if all(lastq(:,ii)==0)
                    all0=[all0 ii];
                end
            end
 
 %%
i=1;
init_theta=0;
xd=way_points(1,i);
yd=way_points(2,i);
zd=way_points(3,i);
    init_theta1=180;
    init_theta2=0;
    init_theta3=0;
    [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
 theta_points=All_theta(:,2); 
 
All_theta=All_theta*pi/180;

what=All_theta(:,2);
% hh=directKinematics(what);
% hhh=ForwardKinematics(what,1);
robot.ApplyPosi2(what);

q = robot.GetState();
while(norm(directKinematics(q) - way_points(:,1)) > 0.1 )
    now_2=directKinematics(q);
    q = robot.GetState();
end
now_3=directKinematics(q)
qd = robot.GetState();

T_tot=(size(way_points,2)-1)*1;
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(way_points,2)-1):T_tot;
% tpts = 0:T_tot/(2-1):T_tot;

% % 续上速度
desired_v32=[zeros(3,size(way_points,2))];
desired_v72=[zeros(7,size(way_points,2))];
[points,points_dot,points_dotdot,pp] = cubicpolytraj(way_points,tpts,tvec,...
                'VelocityBoundaryCondition', desired_v32);   
            
way_points;
[hang,liehh]=size(points);    

for i_inv = 1 : liehh
    init_theta1=180;
    init_theta2=0;
    init_theta3=0;
    xd=points(1,i_inv);
    yd=points(2,i_inv);
    zd=points(3,i_inv);
    [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
%      [All_theta] = inverse_with_gesture_zzz(xd,yd,zd,init_theta1,init_theta2,init_theta3).';
if size(All_theta,2) ~=0
    which_need=abs(All_theta(7,:)) >= 140;
    need_plus=All_theta(7,which_need);
    if need_plus < -140
        All_theta(7,which_need)=need_plus+180;
    elseif need_plus > 140
        All_theta(7,which_need)=need_plus-180;
    end
end 
    count_no=0;
            while isempty(All_theta)
                    if yd<0
                        init_theta2=init_theta2+1;
                        init_theta1=180-count_no;
                    elseif yd>0
                        init_theta2=init_theta2+1;
                        init_theta1=180+count_no;
                    else
                        init_theta2=init_theta2+1;
                        init_theta1=180+count_no;  
                    end
                count_no=count_no+1;
                [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
            end
    [hang,lie]=size(All_theta);
    temp=theta_points(:,end);
    tott=1000;
    delta_matrix=All_theta-temp;
    for each_lie =1:lie
        now=sum(abs(All_theta(:,each_lie)-temp));
        if now < tott
            tott=now;
            which=each_lie;
        end
    end
    to_add=All_theta(:,which);
    to_add(7)=0;
    theta_points=[theta_points to_add];
end
theta_points=theta_points.*pi/180;
theta_points(:,1)=[];
theta=theta_points;
theta_dot=[desired_v72(:,1) (theta(:,3:end)-theta(:,1:end-2))/Ts/2 desired_v72(:,2)];

lastCmdTime=0; t=0;
%% go into the loop
for i=1:size(theta_dot,2)*5
    pause(Ts);
    this_point=ceil(t/T_tot*size(theta_dot,2));
    
            currentCmdTime = robot.GetLastCmdTime();
        dt = (currentCmdTime-lastCmdTime)/1000;
        % get states feedback
    feedback_joint_position=robot.GetState();
    feedback_joint_velocity=robot.GetV();
    robot.ApplyControl(theta_dot(:,this_point+1), Ts); % velocity control
    
        lastCmdTime = currentCmdTime;
        t = t+dt;
end

