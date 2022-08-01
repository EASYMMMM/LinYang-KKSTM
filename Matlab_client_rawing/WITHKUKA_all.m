%% Example of using KST class for interfacing with KUKA iiwa robots
% This example script is used to show how to utilise
% the soft real time control of the KUKA iiwa 7 R 800 for
% Moving first joint of the robot, using a sinusoidal function

% First start the server on the KUKA iiwa controller
% Then run this script using Matlab

% Important: Be careful when runnning the script, be sure that no human, nor obstacles
% are around the robot

% This example works with Sunrise application version KST_1.7  and higher.
% Copyright Mohammad SAFEEA, 8th-August-2018


close all;clear;clc;
warning('off')
%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main')  
raw_points=200;FLAG=0;predict_y=[];
wrong=0;how_many_points_0=[];kinds=12;how_many_points_1=[];
how_many_points_2=[];all_middle=[];
how_many_points_3=[];how_many_points_4=[];how_many_points_5=[];how_many_points_11=[];
how_many_points_6=[];how_many_points_7=[];how_many_points_8=[];how_many_points_9=[];how_many_points_10=[];
    recv_once_0=[];recv_once_1=[];recv_once_2=[];recv_once_3=[];recv_once_4=[];recv_once_5=[];recv_once_6=[];
     recv_once_6=[];recv_once_7=[];recv_once_8=[];recv_once_9=[];recv_once_10=[];recv_once_11=[];recv_once_12=[];current_9xyz=[];
all_9xyz=[];final_EMGxyz=[];all_x_intension=[];all_y_intension=[];all_z_intension=[];all_current_7_avg=[];

flag = 0;count_which_matrix=0;

%% 设置连接参数，要连接的地址为127.0.0.1(即本地主机)，端口号为5174，作为客户机连接。
Client=tcpip('127.0.0.1',5000,'NetworkRole','client');

!(C:\TestCode\Kinect_Matlab\x64\Debug\Kinect_Matlab.exe)&
%% 建立连接，建立完成后进行下一步，否则报错
fopen(Client);%与一个服务器建立连接，直到建立完成返回，否则报错。
sprintf('what is it?')


%% 发送字符串，pause（1）要不要都可以
sendtxt = 'hello hello';count2=0;count_x=0;
fprintf(Client,sendtxt);total_rece_0=[];total_rece_1=[];total_rece_2=[];total_rece_6=[];ALL_LEN_OF_RECV2=[];
total_rece = [];total_head=0;total_rece_3=[];total_rece_4=[];total_rece_5=[];all_delta_theta=[];
all_delta_wan_y=[];all_delta_wan_x=[];all_delta_wan_z=[];
all_delta_jian_y=[];all_delta_jian_x=[];all_delta_jian_z=[];
all_delta_zhou_y=[];all_delta_zhou_x=[];all_delta_zhou_z=[];
all_sum_wan_y=[];all_sum_wan_x=[];all_sum_wan_z=[];




%% Start a connection with the server
flag=iiwa.net_establishConnection();
if flag==0
  return;
end
pause(1);
disp('Moving first joint of the robot using a sinusoidal function')
    

%% Go to initial position
relVel=0.15; % the relative velocity
theta_points=[-0.401300000000000;0.953900000000000;0;-1.04900000000000;0;1.13740000000000;0].';
[-0.401300000000000;0.953900000000000;0;-1.04900000000000;0;1.13740000000000;0]
[-0.405703173982151,0.956061280776520,0,-1.04480238839663,0,1.13946500390074,0]

theta_points2=num2cell(theta_points);all_gamma=[];all_beta=[];all_alpha=[];
iiwa.movePTPJointSpace(theta_points2, relVel); % move to initial configuration

%% Kalman
R = 0.005;  % 测量噪音方差矩阵
kalmanx = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalmany = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalmanz = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalman1 = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalman2 = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalman3 = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
%% Caculate 

%% Start direct servo in joint space       
w=1.5; % motion constants, frequency rad/sec
A=pi/6; % motion constants, amplitude of motion
counter=0;
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];v_filt=[];new_v_filt=[];

theta_points=[ -0.4013,0.9539,0,-1.0490,0,1.1374,0].';sum_e=0;
[-0.401300000000000;0.953900000000000;0;-1.04900000000000;0;1.13740000000000;0]
way_points=[0.65;0.3;0.25];theta_points_final=[];
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

Ts=0.010;
T_tot=30;
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(theta_points,2)-1):T_tot;

[theta,theta_dot,theta_dotdot,pp] = cubicpolytraj(theta_points,tpts,tvec,...
                'VelocityBoundaryCondition', theta_d_des);

%% Start direct servo in joint space       
iiwa.realTime_startVelControlJoints();all_output=[];
% iiwa.realTime_startImpedanceJoints(0.6,0,0,0.1,3500,250,10);

% iiwa.realTime_startVelControlJoints();
counter=0;
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];v_filt=[];new_v_filt=[];
%% Initiate PIDDDDDDDDDDDDDDDDDDDDDDDDDD variables
    k_cartesian = diag([100,100,100]*1*1)*1.3*5/2;
    b_cartesian = diag([100,100,100]*14*0.707*45/1000*0.7*5*1.4/2);   
    H_inv = diag([1 1 1]/10/5*3*2);
    w_n=(k_cartesian.*H_inv)^0.5;
    w_n=w_n(1,1)
    zeta=b_cartesian.*H_inv/2./w_n;
    zeta=zeta(1,1)
    syms xxx; %定义x是一个未知量
    eqn=xxx^2+2*zeta*w_n*xxx+w_n^2==0; % 定义方程，eqn只是一个代号，代表sin(x)==1
    solX=solve(eqn,xxx) % 求方程eqn中的x，放入solX中
    P_coe=diag([100*2,100*2,100*2]/1000*7);
    D_coe=diag([1,1,1]/1000*4);
    I_coe=diag([1,1,1]/1000*6);
    
    %%
    % gain on the orientation difference of the end effector
    k_vel_p = 50;
    % time record
    qmax = [170,120,170,120,170,120,175]*pi/180;
    qmin = -qmax;
    dqlimit = [110,110,128,128,204,184,184]*pi/180;
    t = 0;
    list_x=[];list_v=[];list_a=[];all_xe=[];all_qed=[];all_qedd=[];all_flagg=[];
all_target_end_effector_p_next=[];all_feedback_joint_velocity_after=[];
             new_f=[];new_v=[];new_torque=[];
    all_feedback_joint_position=[];
 %% 
IP_remote = "192.168.11.1"; 
port_remote = 5000;
IP_local = "192.168.11.2"; 
port_local = 5000;
Role = 'client';
t_server = tcpip(IP_remote,port_remote,...
                'NetworkRole',Role,...
                'LocalPort',port_local,...
                'TimeOut',20,...
                'InputBufferSize',8192);

t_server.InputBuffersize=100000;

disp(['未打开！',datestr(now)])
fopen(t_server);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])
%get(t_server);
data_all = [];count_right = 0;
    
    
    
%%  please prepare for starting
while  count2<3500
%    
current_9xyz=[];
    count2 = count2 +1;
tic

            if mod(count2,3000) == 0
                 disp('3000个数过去啦------');
            end
            %%
     if  t_server.BytesAvailable>0
%              t_server.BytesAvailable % 代表缓冲区里累积了多少的字节数的数据/8

%     data_recv = fread(t_server,t_server.BytesAvailable/8,'double');%从缓冲区读取数字数据

    data_recv = fread(t_server,t_server.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
%     disp(size(data_recv));
    count_right = count_right + 1;
    which_head=find(88887<=data_recv);
%     [o,which_head]=min(abs(data_recv-888.888));
    
    which_head2=which_head(end);
    this_frame=data_recv(which_head2:end);
    if length(this_frame)<16
        this_frame=[this_frame; [0 0 0 0 0 0 0].'];
    end
    data_all(:,count_right) = this_frame;
     end
 %%           
       
     pause(0.01);
if Client.BytesAvailable > 0
    recv=fread(Client,Client.BytesAvailable,'char');
    if rem(length(recv),8) == 0
    recv1 = dec2hex(recv,2);
    recv2 = [];
    flagg=0;   

    pass=0;
    for i = 1:length(recv1) 
        if pass == 0;
           if i <= length(recv1)-7 && sum(recv(i:(i+7)) == 122) == 8;
                pass = 7;
                flagg=flagg+1;
            else
                recv2 = [recv2,recv1(i,:)]; %小端模式
            end
        else 
            pass = pass -1;
        end
    end
  
    count = 0;
    if isempty(recv_once_11)
        
    else
        while ~isempty(recv_once_11)  %% enough for 1 column
            current_9xyz=[recv_once_0(1) recv_once_1(1) recv_once_2(1) recv_once_3(1) recv_once_4(1) recv_once_5(1) recv_once_6(1) recv_once_7(1) recv_once_8(1) recv_once_9(1) recv_once_10(1) recv_once_11(1)];
            recv_once_0(1)=[];recv_once_1(1)=[];recv_once_2(1)=[];recv_once_3(1)=[];recv_once_4(1)=[];recv_once_5(1)=[];
            recv_once_6(1)=[];recv_once_7(1)=[];recv_once_8(1)=[];recv_once_9(1)=[];recv_once_10(1)=[];recv_once_11(1)=[];
        end
    end
    
if length(find(recv2=='0'))==length(recv2)
    recv2=[];
end

        while count < length(recv2)/16  
    
        middle = hex2num(recv2(count*16+1:count*16+16));
        all_middle=[all_middle middle];
        count = count + 1;
        
       if  middle ~=0
       count_which_matrix=count_which_matrix+1;

            if rem(count_which_matrix,kinds)-1 == 0;
                how_many_points_0=[how_many_points_0,middle];
                recv_once_0 = [recv_once_0;middle;];
            elseif rem(count_which_matrix,kinds)-1 == 1;
                how_many_points_1=[how_many_points_1,middle];
                recv_once_1 = [recv_once_1;middle;];  
            elseif rem(count_which_matrix,kinds)-1 == 2;
                how_many_points_2=[how_many_points_2,middle];
                recv_once_2 = [recv_once_2;middle;]; 
            elseif rem(count_which_matrix,kinds)-1 == 3;
                how_many_points_3=[how_many_points_3,middle];
                recv_once_3 = [recv_once_3;middle;]; 
            elseif rem(count_which_matrix,kinds)-1 == 4;
                how_many_points_4=[how_many_points_4,middle];
                recv_once_4 = [recv_once_4;middle;]; 
            elseif rem(count_which_matrix,kinds)-1 == 5;
                how_many_points_5=[how_many_points_5,middle];
                recv_once_5 = [recv_once_5;middle;]; 
            elseif rem(count_which_matrix,kinds)-1 == 6;
                how_many_points_6=[how_many_points_6,middle];
                recv_once_6 = [recv_once_6;middle;]; 
            elseif rem(count_which_matrix,kinds)-1 == 7;
                how_many_points_7=[how_many_points_7,middle];
                recv_once_7 = [recv_once_7;middle;]; 
             elseif rem(count_which_matrix,kinds)-1 == 8;
                how_many_points_8=[how_many_points_8,middle];
                recv_once_8 = [recv_once_8;middle;];                
             elseif rem(count_which_matrix,kinds)-1 == 9;
                how_many_points_9=[how_many_points_9,middle];
                recv_once_9 = [recv_once_9;middle;];                
                
              elseif rem(count_which_matrix,kinds)-1 == 10;
                how_many_points_10=[how_many_points_10,middle];
                recv_once_10 = [recv_once_10;middle;];  
                
            elseif rem(count_which_matrix,kinds)-1 == -1;
                how_many_points_11=[how_many_points_11,middle];
                recv_once_11 = [recv_once_11;middle;]; 
            end
       end
            
        end  
    end
end
toc
end

%% CONTROAL LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP
disp(' TIME TO GOOOOO')    
% how_many_points_0=[];how_many_points_1=[];each_update_16=[];
% how_many_points_2=[];how_many_heads_total=0;all_middle=[];
% how_many_points_3=[];how_many_points_4=[];how_many_points_5=[];how_many_points_11=[];
% how_many_points_6=[];how_many_points_7=[];how_many_points_8=[];how_many_points_9=[];how_many_points_10=[];
%     recv_once_0=[];recv_once_1=[];recv_once_2=[];recv_once_3=[];recv_once_4=[];recv_once_5=[];recv_once_6=[];
% recv_once_7=[];recv_once_8=[];recv_once_9=[];recv_once_10=[];recv_once_11=[];recv_once_12=[];current_9xyz=[];
 current_combine=[];predict_x=0;predict_y=0;predict_z=0;
    
    
%% Control loop   
all_v_cartesian_target=[];all_v_cartesian=[];alldt=[];all_contact_force_after=[];
all_target_joint_position_e=[];
All_v=[];EX_force_ori=[];all_F_contact=[];my_t=[{0} {0} {0} {0} {0} {0} {0}];all_f_attractor=[];all_end_effector_p=[];
robot_type=1;  all_jpos=[]; all_jtor=[];my_torque=[0 0 0 0 0 0 0];F_contact=0;all_joint_pos=[];all_target_end_effector_p=[];
all_dt_real=[];
q_t1dd=theta_dotdot(:,1);q_t1d=[0 0 0 0 0 0 0].';q_t1=[0 0 0 0 0 0 0].';intergal_goal=[];adjust=[];
refer_pos=[];all_init_direction_theory=[];all_acc=[];
for q=1:size(theta_dot,2)
         [ pose, nsparam, rconf, jout ] = ForwardKinematics( theta(:,q).', robot_type );
        end_effector_p = pose(1:3,4);
        refer_pos=[refer_pos end_effector_p];
end
init_direction=refer_pos(:,1);init_direction_0=refer_pos(2,1);init_direction_end=refer_pos(2,end);
accum_dt=0;all_xe=[]; all_xde=[];
init_direction_theory=init_direction;

init_pos=theta(:,1);feedback_joint_velocity_after=theta_dot(:,1);all_output=[];
dt_real=Ts;

rate_xdetjia=[];rate_target=[];
target_joint_position_next=[-0.401396453114820,0.954044131099899,7.47815770448098e-05,-1.04905153079427,4.89915666364060e-05,1.13728358894867,-6.17187693413365e-05];
target_joint_velocity_next=[0 0 0 0 0 0 0].';all_this_point=[];last_p=target_joint_position_next;all_xetjia=[];v_filt=[0 0 0].';
count_right
 current_combine=[];predict_x=0;predict_y=0;predict_z=0;
 all_new_EMG2=[];all_new_EMG=[];all_predict_z=[];all_predict_x=[];all_predict_y=[];
 
tic;
for i_theta=1:size(theta_dot,2)
     if  t_server.BytesAvailable>0
%              t_server.BytesAvailable % 代表缓冲区里累积了多少的字节数的数据/8

    data_recv = fread(t_server,t_server.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
%     disp(size(data_recv));
    count_right = count_right + 1;
    which_head=find(88887<=data_recv);
%     [o,which_head]=min(abs(data_recv-888.888));
    which_head2=which_head(end);
    this_frame=data_recv(which_head2:end);
    if length(this_frame)<16
        this_frame=[this_frame; [0 0 0 0 0 0 0].'];
    end
    data_all(:,count_right) = this_frame;
     end

 %%   
if Client.BytesAvailable > 0
    recv=fread(Client,Client.BytesAvailable,'char');
    if rem(length(recv),8) == 0
    recv1 = dec2hex(recv,2);
    recv2 = [];
    flagg=0;   

    pass=0;
    for i = 1:length(recv1) 
        if pass == 0;
           if i <= length(recv1)-7 && sum(recv(i:(i+7)) == 122) == 8;
                pass = 7;
                flagg=flagg+1;
            else
                recv2 = [recv2,recv1(i,:)]; %小端模式
            end
        else 
            pass = pass -1;
        end
    end
  
    count = 0;
    if isempty(recv_once_11)
        
    else
        while ~isempty(recv_once_11)  %% enough for 1 column
            current_9xyz=[recv_once_0(1) recv_once_1(1) recv_once_2(1) recv_once_3(1) recv_once_4(1) recv_once_5(1) recv_once_6(1) recv_once_7(1) recv_once_8(1) recv_once_9(1) recv_once_10(1) recv_once_11(1)];
            recv_once_0(1)=[];recv_once_1(1)=[];recv_once_2(1)=[];recv_once_3(1)=[];recv_once_4(1)=[];recv_once_5(1)=[];
            recv_once_6(1)=[];recv_once_7(1)=[];recv_once_8(1)=[];recv_once_9(1)=[];recv_once_10(1)=[];recv_once_11(1)=[];
        end
    end

        while count < length(recv2)/16  
    
        middle = hex2num(recv2(count*16+1:count*16+16));

        count = count + 1;
        
       if  middle ~=0
       count_which_matrix=count_which_matrix+1;

            if rem(count_which_matrix,kinds)-1 == 0;
                how_many_points_0=[how_many_points_0,middle];
                recv_once_0 = [recv_once_0;middle;];
            elseif rem(count_which_matrix,kinds)-1 == 1;
                how_many_points_1=[how_many_points_1,middle];
                recv_once_1 = [recv_once_1;middle;];  
            elseif rem(count_which_matrix,kinds)-1 == 2;
                how_many_points_2=[how_many_points_2,middle];
                recv_once_2 = [recv_once_2;middle;]; 
            elseif rem(count_which_matrix,kinds)-1 == 3;
                how_many_points_3=[how_many_points_3,middle];
                recv_once_3 = [recv_once_3;middle;]; 
            elseif rem(count_which_matrix,kinds)-1 == 4;
                how_many_points_4=[how_many_points_4,middle];
                recv_once_4 = [recv_once_4;middle;]; 
            elseif rem(count_which_matrix,kinds)-1 == 5;
                how_many_points_5=[how_many_points_5,middle];
                recv_once_5 = [recv_once_5;middle;]; 
            elseif rem(count_which_matrix,kinds)-1 == 6;
                how_many_points_6=[how_many_points_6,middle];
                recv_once_6 = [recv_once_6;middle;]; 
            elseif rem(count_which_matrix,kinds)-1 == 7;
                how_many_points_7=[how_many_points_7,middle];
                recv_once_7 = [recv_once_7;middle;]; 
             elseif rem(count_which_matrix,kinds)-1 == 8;
                how_many_points_8=[how_many_points_8,middle];
                recv_once_8 = [recv_once_8;middle;];                
             elseif rem(count_which_matrix,kinds)-1 == 9;
                how_many_points_9=[how_many_points_9,middle];
                recv_once_9 = [recv_once_9;middle;];                
              elseif rem(count_which_matrix,kinds)-1 == 10;
                how_many_points_10=[how_many_points_10,middle];
                recv_once_10 = [recv_once_10;middle;];      
            elseif rem(count_which_matrix,kinds)-1 == -1;
                how_many_points_11=[how_many_points_11,middle];
                recv_once_11 = [recv_once_11;middle;]; 
            end
       end
            
        end  
    end
end
%%
    
if sum(current_9xyz)~=0 && size(current_combine,1) < 6
    current_combine=[current_combine; current_9xyz;];  %%% each update
elseif sum(current_9xyz)~=0
    current_combine=[current_combine(2:end,:); current_9xyz;];  %%% each update
end

if size(current_combine,1) == 6
predict_z_count=0;predict_x_count=0;predict_y_count=0;current_7_avg=[];
%%
% if isempty(each_update_16)
%     
% else
%  temp_EMG=each_update_16;  % 假设 x:125  y 356 z 3457
% % end
%  each_update_16=[];
%         for each = [1 2 10 4 5 6 7] 
%             each_tongdao=temp_EMG(each:16:end);
%             current_7_avg=[current_7_avg sum(abs(each_tongdao))/length(each_tongdao)];
%         end
%         all_current_7_avg=[all_current_7_avg; current_7_avg;];
%     x_intension=sum(current_7_avg([1 2 5]));
%     y_intension=sum(current_7_avg([3 5 6]));
%     z_intension=sum(current_7_avg([3 4 5 7]));
%     all_x_intension=[all_x_intension; x_intension;];
%     all_y_intension=[all_y_intension; y_intension;];
%     all_y_intension=[all_z_intension; z_intension;];
%% 
        new_EMG=[current_combine];new_EMG2=[current_combine];
        this_sliding=current_combine;
                delta_z=this_sliding(:,12)-this_sliding(:,3);
                delta_x=this_sliding(:,10)-this_sliding(:,1);
                delta_theta=atan2(delta_z,delta_x);
                all_delta_theta=[all_delta_theta; delta_theta*180/pi;];
     for k = 1:3
        if k==1
            who_x=7;
            who_z=9;
            who_y=8;
        elseif k==2
            who_x=4;
            who_z=6;
            who_y=5;
        else
            who_x=1;
            who_z=3;
            who_y=2;            
        end
                rela_wrist_sh_x=this_sliding(:,who_x);
                rela_wrist_sh_z=this_sliding(:,who_z);
              
                    rela_wan_y=this_sliding(:,who_y);

                    output_this_silde_x=[];output_this_silde_z=[];
                    for each_in_silde = 1:size(this_sliding,1)
                        each_rela_jian_z=rela_wrist_sh_z(each_in_silde);
                        each_rela_jian_x=rela_wrist_sh_x(each_in_silde);
                        each_theta=delta_theta(each_in_silde);
%                         each_theta = pi/2-mean(delta_theta);
                        after_x=cos(each_theta)*each_rela_jian_x+sin(each_theta)*each_rela_jian_z;
                        after_z=cos(each_theta)*each_rela_jian_z-sin(each_theta)*each_rela_jian_x;
                        output_this_silde_x=[output_this_silde_x; after_x;];
                        output_this_silde_z=[output_this_silde_z; after_z;];
                    end
                    new_EMG2=[ new_EMG2  output_this_silde_x rela_wan_y output_this_silde_z];    
     end
     new_EMG=[current_combine new_EMG2(:,13:15)-new_EMG2(:,19:21) new_EMG2(:,16:18)-new_EMG2(:,19:21)];
     
     all_new_EMG2=[all_new_EMG2; new_EMG2;];
    all_new_EMG=[all_new_EMG; new_EMG;];
  
                    rela_wan_z=new_EMG(:,22-7)-new_EMG(:,25-7);
                    rela_wan_y=new_EMG(:,21-7)-new_EMG(:,24-7);
                    rela_wan_x=new_EMG(:,20-7)-new_EMG(:,23-7);

                    rela_zhou_z=new_EMG(:,25-7);
                    rela_zhou_y=new_EMG(:,24-7);
                    rela_zhou_x=new_EMG(:,23-7);                    
                    
                    mid_x=floor(length(rela_wan_x)/2);
                    mid_y=floor(length(rela_wan_y)/2);
                    mid_z=floor(length(rela_wan_z)/2);
                    
                    delta_wan_x=-sum(rela_wan_x(1:1))+sum(rela_wan_x(end:end));
                    delta_wan_y=-sum(rela_wan_y(1:1))+sum(rela_wan_y(end:end));
                    delta_wan_z=-sum(rela_wan_z(1:1))+sum(rela_wan_z(end:end));   
                    
                    sum_wan_x=sum(rela_wan_x(1:mid_z))+sum(rela_wan_x(mid_z+1:end));
                    sum_wan_y=sum(rela_wan_y(1:mid_z))+sum(rela_wan_y(mid_z+1:end));
                    sum_wan_z=sum(rela_wan_z(1:mid_z))+sum(rela_wan_z(mid_z+1:end));
 
                    
                    delta_zhou_x=-sum(rela_zhou_x(1:1))+sum(rela_zhou_x(end:end));
                    delta_zhou_y=-sum(rela_zhou_y(1:1))+sum(rela_zhou_y(end:end));
                    delta_zhou_z=-sum(rela_zhou_z(1:1))+sum(rela_zhou_z(end:end));   
                    
                    sum_zhou_x=sum(rela_zhou_x(1:mid_z))+sum(rela_zhou_x(mid_z+1:end));
                    sum_zhou_y=sum(rela_zhou_y(1:mid_z))+sum(rela_zhou_y(mid_z+1:end));
                    sum_zhou_z=sum(rela_zhou_z(1:mid_z))+sum(rela_zhou_z(mid_z+1:end));
                    
                    
                    delta_jian_x=-sum(new_EMG(1:1,1))+sum(new_EMG(end:end,1));
                    delta_jian_y=-sum(new_EMG(1:1,2))+sum(new_EMG(end:end,2));
                    delta_jian_z=-sum(new_EMG(1:1,3))+sum(new_EMG(end:end,3));
                    

        threhold_velocity=0.06;            
                     if delta_wan_x > threhold_velocity || delta_jian_x > threhold_velocity || delta_zhou_x > threhold_velocity
                        predict_x=predict_x+1;
                    elseif delta_wan_x < -threhold_velocity || delta_jian_x < -threhold_velocity || delta_zhou_x < -threhold_velocity
                        predict_x=predict_x-1;
                    else
                        predict_x_count=1;
                     end
                    
                                        
                    if predict_x > 10
                        predict_x=predict_x-1;
                    elseif predict_x <-10
                        predict_x=predict_x+1;                
                    elseif predict_x < 0
                        predict_x=predict_x+predict_x_count;
                    elseif predict_x > 0
                        predict_x=predict_x-predict_x_count;
                    end
                    
                    
                    all_predict_x=[all_predict_x predict_x];
                    
                    if delta_wan_y > threhold_velocity || delta_zhou_y > threhold_velocity  % only motion of wrist related to elbow
                        predict_y=predict_y+1;
                    elseif delta_wan_y < -threhold_velocity || delta_zhou_y < -threhold_velocity
                        predict_y=predict_y-1;
                    else
                        predict_y_count=1;
                    end
                    
                                        
                    if predict_y > 10
                        predict_y=predict_y-1;
                    elseif predict_y <-10
                        predict_y=predict_y+1;
                    elseif predict_y < 0
                        predict_y=predict_y+predict_y_count;
                    elseif predict_z > 0
                        predict_y=predict_y-predict_y_count;
                    end
                    
                    all_predict_y=[all_predict_y predict_y];
                    
                    if delta_zhou_z > threhold_velocity || delta_jian_z > threhold_velocity
                        predict_z=predict_z+1;
                    elseif delta_zhou_z < -threhold_velocity || delta_jian_z < -threhold_velocity
                        predict_z=predict_z-1;
                    else
                        predict_z_count=1;
                    end
                               
                    if predict_z > 10
                        predict_z=predict_z-1;
                    elseif predict_z <-10
                        predict_z=predict_z+1;
                    elseif predict_z < 0
                        predict_z=predict_z+predict_z_count;
                    elseif predict_z > 0
                        predict_z=predict_z-predict_z_count;
                    end

                    all_predict_z=[all_predict_z predict_z];  
                    
                     all_delta_zhou_x=[all_delta_zhou_x delta_zhou_x];
                    all_delta_zhou_y=[all_delta_zhou_y delta_zhou_y];
                    all_delta_zhou_z=[all_delta_zhou_z delta_zhou_z]; 
                    
                    all_delta_jian_x=[all_delta_jian_x delta_jian_x];
                    all_delta_jian_y=[all_delta_jian_y delta_jian_y];
                    all_delta_jian_z=[all_delta_jian_z delta_jian_z]; 
                    
                    all_delta_wan_x=[all_delta_wan_x delta_wan_x];
                    all_delta_wan_y=[all_delta_wan_y delta_wan_y];
                    all_delta_wan_z=[all_delta_wan_z delta_wan_z]; 

                    all_sum_wan_x=[all_sum_wan_x sum_wan_x];
                    all_sum_wan_y=[all_sum_wan_y sum_wan_y];
                    all_sum_wan_z=[all_sum_wan_z sum_wan_z];
end





 %% CONTROL   
    if i_theta == 1
        start = toc
    end
    accum_dt=accum_dt+dt_real;
    all_dt_real=[all_dt_real dt_real];
    dt=dt_real;
    
    
[y,iy]=min(abs(refer_pos(2,:)-init_direction(2)));
%     ratio=(init_direction-init_direction_0)/(init_direction_end-init_direction_0);
    this_point=ceil(accum_dt/T_tot*size(theta_dot,2));
    
    real_point=iy;
    if real_point >= size(theta_dot,2)
        real_point = size(theta_dot,2)-1;
    end
    if this_point >= size(theta_dot,2)
        this_point = size(theta_dot,2)-1;
    end
    all_this_point=[all_this_point this_point];

if i_theta < 20
this_point=real_point;
end
    target_joint_position=target_joint_position_next;
    target_joint_position_next=theta(:,this_point+1).';
    target_joint_velocity=target_joint_velocity_next;
    target_joint_velocity_next=theta_dot(:,this_point+1);  
    
    this_p=iiwa.getJointsPos();;
    this_p=cell2mat(this_p);
    
    all_jpos=[all_jpos; this_p;];  
    
%% caculate outer force

    [ pose_c, nsparam_c, rconf_c, jout_c ] = ForwardKinematics( target_joint_position.', robot_type );
    target_end_effector_p = pose_c(1:3,4);
    target_end_effector_r = pose_c(1:3,1:3);
    all_target_end_effector_p=[all_target_end_effector_p target_end_effector_p];  %-----------
    
    %% ========Adj

% time update
        [Jac_this,A_mat_products] = Jacobian(target_joint_position,robot_type);
        J_dx_dq_this = Jac_this(1:3,:);
    [ pose_c2, nsparam_c, rconf_c, jout_c ] = ForwardKinematics( target_joint_position_next.', robot_type );
    target_end_effector_p_next = pose_c2(1:3,4);
    all_target_end_effector_p_next=[all_target_end_effector_p_next target_end_effector_p_next];
    feedback_joint_position=this_p;
   
        all_feedback_joint_position=[all_feedback_joint_position; feedback_joint_position;];
        contact_force = F_contact;
%         % controller
        [Jac,A_mat_products] = Jacobian(feedback_joint_position,robot_type);
        J_dx_dq = Jac(1:3,:);
        
my_torque=cell2mat(my_t).';
new_torque=[new_torque; cell2mat(my_t);];

    F_contact=1*pinv(J_dx_dq.')*my_torque;
    all_F_contact=[all_F_contact F_contact];       %%!!! 
F_filtered1 = kalmanx(F_contact(1));
F_filtered2 = kalmany(F_contact(2));
F_filtered3 = kalmanz(F_contact(3));
  
F_filt=[F_filtered1 F_filtered2 F_filtered3].';
new_f=[new_f F_filt];


         [ pose, nsparam, rconf, jout ] = ForwardKinematics( feedback_joint_position, robot_type );
        end_effector_p = pose(1:3,4);
        all_end_effector_p=[all_end_effector_p end_effector_p];
        
        end_effector_r = pose(1:3,1:3);


        f_bias=1*[0 0 0].';
        f_bias=end_effector_r*f_bias;
%         projection=end_effector_r*contact_force';
        contact_force_after = (F_filt-f_bias);
flagg=0;      
 thre=2;div=1;
 if abs(contact_force_after(1))<thre
     contact_force_after(1)=0;
     flagg=flagg+1;
 elseif contact_force_after(1)>=thre
     contact_force_after(1)=contact_force_after(1)-thre;
     contact_force_after(1)=contact_force_after(1)/div;
 elseif contact_force_after(1)<=-thre
     contact_force_after(1)=contact_force_after(1)+thre;
     contact_force_after(1)=contact_force_after(1)/div;
 end
 
  if abs(contact_force_after(2))<thre
     contact_force_after(2)=0;
     flagg=flagg+1;
 elseif contact_force_after(2)>=thre
     contact_force_after(2)=contact_force_after(2)-thre;
     contact_force_after(2)=contact_force_after(2)/div;
 elseif contact_force_after(2)<=-thre
     contact_force_after(2)=contact_force_after(2)+thre;
     contact_force_after(2)=contact_force_after(2)/div;
  end
 
  if abs(contact_force_after(3))<thre
     contact_force_after(3)=0;
     flagg=flagg+1;
 elseif contact_force_after(3)>=thre
     contact_force_after(3)=contact_force_after(3)-thre;
     contact_force_after(3)=contact_force_after(3)/div;
 elseif contact_force_after(3)<=-thre
     contact_force_after(3)=contact_force_after(3)+thre;
     contact_force_after(3)=contact_force_after(3)/div;
  end
    
 if i <10
     contact_force_after(3)=0;
 end

  
   all_contact_force_after=[all_contact_force_after contact_force_after];
%    contact_force_after=[0 0 0].';



        v_cartesian_target=J_dx_dq*target_joint_velocity;
        all_v_cartesian_target=[all_v_cartesian_target v_cartesian_target];
   
        
        v_cartesian = J_dx_dq*feedback_joint_velocity_after;
        all_v_cartesian=[all_v_cartesian v_cartesian];

  
% v_filt=[v_filtered1 v_filtered2 v_filtered3].';
new_v_filt=[new_v_filt v_filt];




%% SVD
% [U,S,V] = svd(J_dx_dq)

%         max=4;   % 8  8 5
%         k=max/150; 
%         if i <=1000 && i>850
%             contact_force_after=[k*(i-850) k*(i-850) 0].';
%         elseif i >1000 && i<=1700
%             contact_force_after=[max max 0].';
%         elseif i >1700 && i<=1850
%             contact_force_after=[max-k*(i-1700) max-k*(i-1700) 0].';
% %             contact_force_after=[0 0 0].';
%         end
% %         contact_force_after=[8 0 0].';
%         adjust=[adjust contact_force_after];        

%         max=12;
%         k=max/1000; 
%         if i <=1000
%             contact_force_after=[0 k*i 0].';
%         elseif i <=2000
%             contact_force_after=[0 max-k*(i-1000) 0].';
%         else
%             contact_force_after=[0 0 0].';
% %             contact_force_after=[0 0 0].';
%         end
% %         contact_force_after=[8 0 0].';
%         adjust=[adjust contact_force_after];
        
%         max=12;
%         k=max/1000; 
%         if i <=1000
%             contact_force_after=[0 0 k*i].';
%         else
%             contact_force_after=[0 0 max-k*(i-1000)].';
% %             contact_force_after=[0 0 0].';
%         end
% %         contact_force_after=[8 0 0].';
%         adjust=[adjust contact_force_after];        

%% 33333333333333333333333333333333333333333333333333333333333333333333333333333333

    a_d=(J_dx_dq*target_joint_velocity_next-J_dx_dq*target_joint_velocity)/dt;
    
%        xe=-target_end_effector_p+end_effector_p; %3 1
%        xde=-J_dx_dq*target_joint_velocity+v_filt;
       
       xe=-target_end_effector_p+end_effector_p+J_dx_dq*target_joint_velocity*dt; %3 1
       xde=-J_dx_dq*target_joint_velocity+v_filt+a_d*dt;       
       
       all_xe=[all_xe xe];
       all_xde=[all_xde xde];
       
      x_t1dd=H_inv*(contact_force_after-k_cartesian*xe-b_cartesian*xde);
      
        equal_F=(-k_cartesian*xe-b_cartesian*xde);
        all_f_attractor=[all_f_attractor equal_F];
        
        
     xdetjia=-J_dx_dq*target_joint_velocity+v_filt+x_t1dd*dt;  %%% 
     all_acc=[all_acc xdetjia];
     
     v_filt=xdetjia+J_dx_dq*target_joint_velocity_next;
     rate_xdetjia=[rate_xdetjia xdetjia];
     rate_target=[rate_target J_dx_dq*target_joint_velocity_next];
     
     feedback_joint_velocity_after=pinv(Jac)*[v_filt; 0; 0; 0;];
%      feedback_joint_velocity_after=pinv(J_dx_dq)*v_filt;
  
       feedback_joint_velocity_after = satdq(dqlimit,feedback_joint_velocity_after);
     %% SAFE
    future_pos_7=this_p.'+target_joint_velocity_next*dt;
    for each_joint = 1:7
        if (future_pos_7(each_joint) <= qmin(each_joint)) || (future_pos_7(each_joint) >= qmax(each_joint))
            disp('ERROR ! range is out of limit')
            feedback_joint_velocity_after(each_joint)=0;
        end
    end
    future_pos_3=end_effector_p+J_dx_dq*target_joint_velocity_next*dt;
    if future_pos_3(3)<0.05
        disp('ERROR ! range is out of limit')
        break
    end

    all_feedback_joint_velocity_after=[all_feedback_joint_velocity_after feedback_joint_velocity_after];

    this_theta_matrix=target_joint_velocity_next.';
    this_theta=num2cell(this_theta_matrix);

    this_zukang=num2cell(feedback_joint_velocity_after);

    if i_theta < 20
        my_t=iiwa.sendJointsVelocitiesExTorques(this_zukang);
    else 
        my_t=iiwa.sendJointsVelocitiesExTorques(this_zukang);
    end
    
    t0=toc;
    dt_real=-start+t0;
    start=t0;
    
    
    init_pos=this_p.';
    init_direction=end_effector_p;
    
    fuck_total=J_dx_dq*feedback_joint_velocity_after*dt;
    init_direction_theory=init_direction_theory+fuck_total;
    all_init_direction_theory=[all_init_direction_theory init_direction_theory];    
    
%     iiwa.realTime_stopImpedanceJoints()
end

%% turn off server
% iiwa.realTime_stopImpedanceJoints()
iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer()


refer_pos2=refer_pos.';
disp('Direct servo motion completed successfully')
warning('on')
figure(9);plot(refer_pos2(:,1)); hold on; plot(all_end_effector_p(1,:));
legend('goal','real')
figure(10);plot(refer_pos2(:,2)); hold on; plot(all_end_effector_p(2,:));
legend('goal','real')
figure(11);plot(refer_pos2(:,3)); hold on; plot(all_end_effector_p(3,:));
legend('goal','real')

figure(13);plot(rate_target(1,:)); hold on; plot(rate_xdetjia(1,:)); hold on; plot(new_v_filt(1,:))
figure(14);plot(rate_target(2,:)); hold on; plot(rate_xdetjia(2,:)); hold on; plot(new_v_filt(2,:))
figure(15);plot(rate_target(3,:)); hold on; plot(rate_xdetjia(3,:)); hold on; plot(new_v_filt(3,:))

figure(17);plot(all_dt_real)


figure(59);
plot(new_f(3,:)); hold on;
plot(all_contact_force_after(3,:)); hold on;
plot(all_F_contact(3,:)); hold on; 
plot(all_acc(3,:)*100); hold on; 
plot(new_v_filt(3,:)*300); hold on;
plot(all_init_direction_theory(3,:)*100-20); hold on; 
plot(all_end_effector_p(3,:)*100-20);
legend('KKKK','filter force','Force','acceleration','velocity','theortic position','real position')



pause(20)
fclose(t_server);


                    all=[all_delta_wan_x; all_delta_wan_y; all_delta_wan_z;
                       all_delta_zhou_x; all_delta_zhou_y; all_delta_zhou_z;
                       all_delta_jian_x; all_delta_jian_y; all_delta_jian_z;].';

% legend('goal','real')
%          clust = parcluster('local');
%          job1 = createJob(clust); %开启一个job
%          disp('saving------');
%          temp = all_v_cartesian.';
%          createTask(job1,@mytxt,1,{temp});%再给job1分配一个‘mytxt’的task
%          submit(job1);
% figure;plot(all_target_end_effector_p(1,:));hold on;
% figure;plot(all_target_end_effector_p(1,:)-all_end_effector_p(1,:))


