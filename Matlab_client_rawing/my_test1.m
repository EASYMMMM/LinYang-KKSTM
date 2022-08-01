close all,clear all;clc;
warning('off');

%%

theta_points=[ -0.4013,0.9539,0,-1.0490,0,1.1374,-0.3927].';
way_points=[[0.65;-0.3;0.25]];
[useless, len_way_points]=size(way_points);
 theta_points=theta_points*180/pi;
 
  [hang,lie]=size(way_points);
for i = 1 : lie
    init_theta1=180;
    init_theta2=0;
    xd=way_points(1,i);
    yd=way_points(2,i);
    zd=way_points(3,i);
    [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
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
        now=sum(abs(All_theta(:,each_lie)-temp))
        if now < tott;
            tott=now;
            which=each_lie;
        end
        
    end
    to_add=All_theta(:,which);
    to_add(7)=0;
    theta_points=[theta_points to_add];
end
theta_points=theta_points.*pi/180
theta_d_des=[[0;0;0;0;0;0;0]];
for z =1:len_way_points
    theta_d_des=[theta_d_des [0;0;0;0;0;0;0]];
%     theta_d_des = [[0;0;0;0;0;0;0],[0;0;0;0;0;0;0],[0;0;0;0;0;0;0],[0;0;0;0;0;0;0]];
end

Ts=0.002;
T_tot=20;
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(theta_points,2)-1):T_tot;

[theta,theta_dot,theta_dotdot,pp] = cubicpolytraj(theta_points,tpts,tvec,...
                'VelocityBoundaryCondition', theta_d_des);

            
%%


% %% Create the robot object
% ip='172.31.1.147'; % The IP of the controller
% arg1=KST.LBR7R800; % choose the robot iiwa7R800 or iiwa14R820
% arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
% Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
% iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
% addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main')  
% 
% %% Start a connection with the server
% flag=iiwa.net_establishConnection();
% if flag==0
%   return;
% end
% pause(1);
% disp('Moving first joint of the robot using a sinusoidal function')
%     
% 
% %% Go to initial position
% relVel=0.15; % the relative velocity
% theta_points=[ -0.4013,0.9539,0,-1.0490,0,1.1374,-0.3927];
% way_points=[[0.65;-0.3;0.25],[0.65;-0.3;0.475],[0.6;0;0.475],[0.65;0.3;0.475],[0.65;-0.3;0.25]];
% 
% theta_points2=num2cell(theta_points);
% iiwa.movePTPJointSpace(theta_points2, relVel); % move to initial configuration






