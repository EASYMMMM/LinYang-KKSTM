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

draw_points=200;FLAG=0;predict_y=[];
wrong=0;how_many_points_0=[];kinds=12;how_many_points_1=[];
how_many_points_2=[];how_many_heads_total=0;all_middle=[];
how_many_points_3=[];how_many_points_4=[];how_many_points_5=[];how_many_points_11=[];
how_many_points_6=[];how_many_points_7=[];how_many_points_8=[];how_many_points_9=[];how_many_points_10=[];
    recv_once_0=[];recv_once_1=[];recv_once_2=[];recv_once_3=[];recv_once_4=[];recv_once_5=[];recv_once_6=[];
     recv_once_6=[];recv_once_7=[];recv_once_8=[];recv_once_9=[];recv_once_10=[];recv_once_11=[];recv_once_12=[];current_9xyz=[];
all_9xyz=[];final_EMGxyz=[];

flag = 0;count_which_matrix=0;

%% 设置连接参数，要连接的地址为127.0.0.1(即本地主机)，端口号为5174，作为客户机连接。
Client=tcpip('127.0.0.1',5000,'NetworkRole','client');

!(C:\TestCode\Kinect_Matlab\x64\Debug\Kinect_Matlab.exe)&
%% 建立连接，建立完成后进行下一步，否则报错
fopen(Client);%与一个服务器建立连接，直到建立完成返回，否则报错。
sprintf('what is it?')


%% 发送字符串，pause（1）要不要都可以
sendtxt = 'hello hello';total_rece=[];count2=0;count_x=0;
fprintf(Client,sendtxt);total_rece_0=[];total_rece_1=[];total_rece_2=[];total_rece_6=[];ALL_LEN_OF_RECV2=[];
total_rece = [];total_head=0;total_rece_3=[];total_rece_4=[];total_rece_5=[];



%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR7R800; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main')  

%% Start a connection with the server
flag=iiwa.net_establishConnection();
if flag==0
  return;
end
pause(1);
disp('Moving first joint of the robot using a sinusoidal function')
    

%% Go to initial position
relVel=0.15; % the relative velocity
theta_points=[ -0.4013,0.9539,0,-1.0490,0,1.1374,0];
theta_points2=num2cell(theta_points);
iiwa.movePTPJointSpace(theta_points2, relVel); % move to initial configuration

%% Kalman
R = 0.05;  % 测量噪音方差矩阵
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
all_delta_wan_y=[];all_delta_wan_x=[];all_delta_wan_z=[];
all_sum_wan_y=[];all_sum_wan_x=[];all_sum_wan_z=[];

theta_points=[ -0.4013,0.9539,0,-1.0490,0,1.1374,0].';sum_e=0;
way_points=[0.65;0.3;0.25];theta_points_final=[];all_dttt=[];
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

Ts=20/3000;
T_tot=20;all_predict_y=[];all_predict_x=[];all_predict_z=[];all_new_EMG=[];
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(theta_points,2)-1):T_tot;

[theta,theta_dot,theta_dotdot,pp] = cubicpolytraj(theta_points,tpts,tvec,...
                'VelocityBoundaryCondition', theta_d_des);

%% Start direct servo in joint space       
iiwa.realTime_startDirectServoJoints();all_output=[];
% iiwa.realTime_startImpedanceJoints(0.6,0,0,0.1,3500,250,10);

% iiwa.realTime_startVelControlJoints();

w=1.5; % motion constants, frequency rad/sec
A=pi/6; % motion constants, amplitude of motion
counter=0;
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];v_filt=[];new_v_filt=[];all_delta_theta=[];
%% Initiate PIDDDDDDDDDDDDDDDDDDDDDDDDDD variables
k=[0.9089 0.8639 0.324];
k1=1;
k2=1.5;
k3=1;

dt=0;
tic;
t0=toc; % calculate initial time
    k_cartesian = diag([100*k1,100*k2,100*k3]*2);
    b_cartesian = diag([100,100,100]*2*0.707*15/1000);   
    H_inv = diag([1*k1 1*2 1*k3]*100*0.9);
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
   %%
   coef_v_order9=[
    0.09637703954237,  0.09877664875597,   0.1005959487483,   0.1018182036649,...
     0.1024321592885,   0.1024321592885,   0.1018182036649,   0.1005959487483,...
    0.09877664875597,  0.09637703954237
].';


  coef_f_order26=[
    0.02718719168633,  0.02960040617057,   0.0319283543919,  0.03414443214987,...
    0.03622288837122,  0.03813922211286,  0.03987056397838,  0.04139603507008,...
    0.04269707696676,  0.04375774671473,  0.04456497143587,  0.04510875788065,...
    0.04538235307079,  0.04538235307079,  0.04510875788065,  0.04456497143587,...
    0.04375774671473,  0.04269707696676,  0.04139603507008,  0.03987056397838,...
    0.03813922211286,  0.03622288837122,  0.03414443214987,   0.0319283543919,...
    0.02960040617057,  0.02718719168633
].';

             new_f=[];new_v=[];new=[];
    all_feedback_joint_position=[];
%% Control loop   
all_v_cartesian_target=[];all_v_cartesian=[];alldt=[];all_contact_force_after=[];
last_p=[-0.4013,0.9539,0,-1.0490,0,1.1374,-0.3927];all_target_joint_position_e=[];
All_v=[];EX_force_ori=[];all_F_contact=[];my_t=[{0} {0} {0} {0} {0} {0} {0}];all_f_attractor=[];all_end_effector_p=[];
robot_type=1;  all_jpos=[]; all_jtor=[];my_torque=[0 0 0 0 0 0 0];F_contact=0;all_joint_pos=[];all_target_end_effector_p=[];
all_dt_real=[];
q_t1dd=theta_dotdot(:,1);q_t1d=[0 0 0 0 0 0 0].';q_t1=[0 0 0 0 0 0 0].';intergal_goal=[];adjust=[];
pause(1);

%%
while  count2<3500
%    
current_9xyz=[];
    count2 = count2 +1;

            if mod(count2,3000) == 0
                 disp('3000个数过去啦------');
            end
            
 %%           
       
     pause(0.001);
if Client.BytesAvailable > 0
    
 
    recv=fread(Client,Client.BytesAvailable,'char');
    
    if rem(length(recv),8) == 0
    recv1 = dec2hex(recv);
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
%     end
    end
end

end

%% CONTROAL LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP
disp(' TIME TO GOOOOO')
how_many_points_0=[];how_many_points_1=[];
how_many_points_2=[];how_many_heads_total=0;all_middle=[];
how_many_points_3=[];how_many_points_4=[];how_many_points_5=[];how_many_points_11=[];
how_many_points_6=[];how_many_points_7=[];how_many_points_8=[];how_many_points_9=[];how_many_points_10=[];
%     recv_once_0=[];recv_once_1=[];recv_once_2=[];recv_once_3=[];recv_once_4=[];recv_once_5=[];recv_once_6=[];
% recv_once_7=[];recv_once_8=[];recv_once_9=[];recv_once_10=[];recv_once_11=[];recv_once_12=[];current_9xyz=[];

%     zeros(M,N) 
     current_combine=[];
for i=1:size(theta_dot,2)-1
%     global each_update_16
%     target_joint_position=theta(:,1).';
%     target_joint_position_next=theta(:,1).';
%     target_joint_dotdot=theta_dotdot(:,1);
%     target_joint_velocity=theta_dot(:,1);
%     target_joint_velocity_next=theta_dot(:,1);
     predict_x=0;predict_y=0;predict_z=0;
           
    target_joint_position=theta(:,i).';
    target_joint_position_next=theta(:,i+1).';
    target_joint_dotdot=theta_dotdot(:,i);
    target_joint_velocity=theta_dot(:,i);
    target_joint_velocity_next=theta_dot(:,i+1);    

    time=toc;
    dt_real=time-t0;
    all_dt_real=[all_dt_real dt_real];
    dt=Ts;
    t0=toc;
    counter=counter+1;
    
    this_p=iiwa.getJointsPos();
    
    

    all_jpos=[all_jpos; this_p;];
    
    
    this_p=cell2mat(this_p);
    velocity=(this_p-last_p)/dt;
    All_v=[All_v; velocity;];
    last_p=this_p;

    
    current_9xyz=[];   
    
  

if Client.BytesAvailable > 0
    recv=fread(Client,Client.BytesAvailable,'char');
    if rem(length(recv),8) == 0
    recv1 = dec2hex(recv);
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
%     if isempty(recv_once_11)
%         
%     else
        while ~isempty(recv_once_11)  %% enough for 1 column
            current_9xyz=[recv_once_0(1) recv_once_1(1) recv_once_2(1) recv_once_3(1) recv_once_4(1) recv_once_5(1) recv_once_6(1) recv_once_7(1) recv_once_8(1) recv_once_9(1) recv_once_10(1) recv_once_11(1)];
          
            recv_once_0(1)=[];recv_once_1(1)=[];recv_once_2(1)=[];recv_once_3(1)=[];recv_once_4(1)=[];recv_once_5(1)=[];
            recv_once_6(1)=[];recv_once_7(1)=[];recv_once_8(1)=[];recv_once_9(1)=[];recv_once_10(1)=[];recv_once_11(1)=[];
        end
%     end
    
if length(find(recv2=='0'))==length(recv2)
    recv2=[];
end



        
            how_many_nums=length(recv2)/16;
            if rem(how_many_nums,12) == 0
                dele=floor(how_many_nums/12)-1;
            else
                dele=floor(how_many_nums/12);
            end
            now_many_nums=how_many_nums-12*dele;
ALL_LEN_OF_RECV2=[ALL_LEN_OF_RECV2 now_many_nums];            

       while count < now_many_nums
           middle = hex2num(recv2(end-(now_many_nums-count-1)*16-16+1:end-(now_many_nums-count-1)*16));
%        middle = hex2num(recv2(count*16+1:count*16+16));
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
%     end
    end
end

if sum(current_9xyz)~=0 && size(current_combine,1) < 6
    current_combine=[current_combine; current_9xyz;];  %%% each update
elseif sum(current_9xyz)~=0
    current_combine=[current_combine(2:end,:); current_9xyz;];  %%% each update
end

if size(current_combine,1) == 6


%% 
        new_EMG=[current_combine];new_EMG2=[current_combine];
        this_sliding=current_combine;
                delta_z=this_sliding(:,12)-this_sliding(:,3);
                delta_x=this_sliding(:,10)-this_sliding(:,1);
                delta_theta=atan(delta_z./delta_x);
                all_delta_theta=[all_delta_theta; delta_theta*180/pi;];
     for k = 1:3
        if k==1
            who_x=14-7;
            who_z=16-7;
            who_y=15-7;
        elseif k==2
            who_x=11-7;
            who_z=13-7;
            who_y=12-7;
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
%                         each_theta = 45*pi/180;
                        after_x=cos(each_theta)*each_rela_jian_x+sin(each_theta)*each_rela_jian_z;
                        after_z=cos(each_theta)*each_rela_jian_z-sin(each_theta)*each_rela_jian_x;
                        output_this_silde_x=[output_this_silde_x; after_x;];
                        output_this_silde_z=[output_this_silde_z; after_z;];
                    end
                    new_EMG2=[ new_EMG2  output_this_silde_x rela_wan_y output_this_silde_z];    
     end
     new_EMG=[current_combine new_EMG2(:,13:15)-new_EMG2(:,19:21) new_EMG2(:,16:18)-new_EMG2(:,19:21)];
     
     
    all_new_EMG=[all_new_EMG; new_EMG;];
  
                    rela_wan_z=new_EMG(:,22-7);
                    rela_wan_y=new_EMG(:,21-7);
                    rela_wan_x=new_EMG(:,20-7);

                    rela_zhou_z=new_EMG(:,25-7);
                    rela_zhou_y=new_EMG(:,24-7);
                    rela_zhou_x=new_EMG(:,23-7);                    
                    
                    mid_x=floor(length(rela_wan_x)/2);
                    mid_y=floor(length(rela_wan_y)/2);
                    mid_z=floor(length(rela_wan_z)/2);
                    
                    delta_wan_x=-sum(rela_wan_x(1:mid_z))+sum(rela_wan_x(mid_z+1:end));
                    delta_wan_y=-sum(rela_wan_y(1:mid_z))+sum(rela_wan_y(mid_z+1:end));
                    delta_wan_z=-sum(rela_wan_z(1:mid_z))+sum(rela_wan_z(mid_z+1:end));
                    
                    
                    sum_wan_x=sum(rela_wan_x(1:mid_z))+sum(rela_wan_x(mid_z+1:end));
                    sum_wan_y=sum(rela_wan_y(1:mid_z))+sum(rela_wan_y(mid_z+1:end));
                    sum_wan_z=sum(rela_wan_z(1:mid_z))+sum(rela_wan_z(mid_z+1:end));
                    
                    delta_jian_x=-sum(new_EMG(1:mid_z,1))+sum(new_EMG(mid_z+1:end,1));
                    delta_jian_y=-sum(new_EMG(1:mid_z,2))+sum(new_EMG(mid_z+1:end,2));
                    delta_jian_z=-sum(new_EMG(1:mid_z,3))+sum(new_EMG(mid_z+1:end,3));
                    
                    if delta_wan_x > 0.05 || delta_jian_x > 0.05
                        predict_x=predict_x+1;
                    elseif delta_wan_x < -0.05 || delta_jian_x < -0.05
                        predict_x=predict_x-1;
                    end
                    all_predict_x=[all_predict_x predict_x];
                    
                    if delta_wan_y > 0.05 || delta_jian_y > 0.05
                        predict_y=predict_y+1;
                    elseif delta_wan_y < -0.05 || delta_jian_y < -0.05
                        predict_y=predict_y-1;
                    end
                    all_predict_y=[all_predict_y predict_y];
                    
                    if delta_wan_z > 0.05 || delta_jian_z > 0.05
                        predict_z=predict_z+1;
                    elseif delta_wan_z < -0.05 || delta_jian_z < -0.05
                        predict_z=predict_z-1;
                    end
                    all_predict_z=[all_predict_z predict_z];                    
                    
                    all_delta_wan_x=[all_delta_wan_x delta_wan_x];
                    all_delta_wan_y=[all_delta_wan_y delta_wan_y];
                    all_delta_wan_z=[all_delta_wan_z delta_wan_z]; 
%                     all=[all_delta_wan_x; all_delta_wan_y; all_delta_wan_z;].';
                    all_sum_wan_x=[all_sum_wan_x sum_wan_x];
                    all_sum_wan_y=[all_sum_wan_y sum_wan_y];
                    all_sum_wan_z=[all_sum_wan_z sum_wan_z];
end


%% caculate outer force

    [ pose_c, nsparam_c, rconf_c, jout_c ] = ForwardKinematics( target_joint_position.', robot_type );
    target_end_effector_p = pose_c(1:3,4);
    target_end_effector_r = pose_c(1:3,1:3);
%     all_target_end_effector_p=[all_target_end_effector_p target_end_effector_p];  %-----------
    
    %% ========Adj
%     R=pose_c(1:3,1:3); p=pose_c(1:3,4)
%     p1=p(1); p3=p(3); p2=p(2);
%     p_use=[0 -p3 p2; p3 0 -p1; -p2 p1 0];
%     Ad=[R [0 0 0; 0 0 0; 0 0 0]];
%     Ad=[Ad; [p_use*R R]];
%     inv_Ad=inv(Ad);
% time update
        [Jac_this,A_mat_products] = Jacobian(target_joint_position,robot_type);
        J_dx_dq_this = Jac_this(1:3,:);

        
        [Jac_next,A_mat_products] = Jacobian(target_joint_position_next,robot_type);
        J_dx_dq_next = Jac_next(1:3,:);
        
    [ pose_c2, nsparam_c, rconf_c, jout_c ] = ForwardKinematics( target_joint_position_next.', robot_type );
    target_end_effector_p_next = pose_c2(1:3,4);


        % get states feedback
    feedback_joint_position=this_p;
        lastConfiguration = feedback_joint_position;
%         all_feedback_joint_position=[all_feedback_joint_position; feedback_joint_position;];



    
    
        contact_force = F_contact;
%         % controller
        [Jac,A_mat_products] = Jacobian(feedback_joint_position,robot_type);
        J_dx_dq = Jac(1:3,:);
        
my_torque=cell2mat(my_t).';
% new=[new; cell2mat(my_t);];
    

    feedback_joint_velocity=velocity.';

    F_contact=1*pinv(J_dx_dq.')*my_torque;
%     all_F_contact=[all_F_contact F_contact];       %%!!! 
        
        
         
F_filtered1 = kalmanx(F_contact(1));
F_filtered2 = kalmany(F_contact(2));
F_filtered3 = kalmanz(F_contact(3));
  
F_filt=[F_filtered1 F_filtered2 F_filtered3].';
% new_f=[new_f F_filt];


         [ pose, nsparam, rconf, jout ] = ForwardKinematics( feedback_joint_position, robot_type );
        end_effector_p = pose(1:3,4);
        all_end_effector_p=[all_end_effector_p end_effector_p];
        
        end_effector_r = pose(1:3,1:3);
        
 %         f_bias=[10 2 15].';
        f_bias=1*[0 0 5].';
        f_bias=end_effector_r*f_bias;
%         projection=end_effector_r*contact_force';
        contact_force_after = (F_filt-f_bias);
 flagg=0;      
 thre=3;
 if abs(contact_force_after(1))<thre
     contact_force_after(1)=0;
     flagg=flagg+1;
 elseif contact_force_after(1)>=thre
     contact_force_after(1)=contact_force_after(1)-thre;
     contact_force_after(1)=contact_force_after(1)/3;
 elseif contact_force_after(1)<=-thre
     contact_force_after(1)=contact_force_after(1)+thre;
     contact_force_after(1)=contact_force_after(1)/3;
 end
 
  if abs(contact_force_after(2))<thre
     contact_force_after(2)=0;
     flagg=flagg+1;
 elseif contact_force_after(2)>=thre
     contact_force_after(2)=contact_force_after(2)-thre;
     contact_force_after(2)=contact_force_after(2)/3;
 elseif contact_force_after(2)<=-thre
     contact_force_after(2)=contact_force_after(2)+thre;
     contact_force_after(2)=contact_force_after(2)/3;
  end
 
  if abs(contact_force_after(3))<thre
     contact_force_after(3)=0;
     flagg=flagg+1;
 elseif contact_force_after(3)>=thre
     contact_force_after(3)=contact_force_after(3)-thre;
     contact_force_after(3)=contact_force_after(3)/3;
 elseif contact_force_after(3)<=-thre
     contact_force_after(3)=contact_force_after(3)+thre;
     contact_force_after(3)=contact_force_after(3)/3;
  end
    

  
 thre_2=4;
  if contact_force_after(3)>thre_2
      contact_force_after(3)=thre_2;
  elseif contact_force_after(3)<-thre_2
       contact_force_after(3)=-thre_2;
  end
  if contact_force_after(1)>thre_2
      contact_force_after(1)=thre_2;
  elseif contact_force_after(1)<-thre_2
       contact_force_after(1)=-thre_2;
  end 
   if contact_force_after(2)>thre_2
      contact_force_after(2)=thre_2;
  elseif contact_force_after(2)<-thre_2
       contact_force_after(2)=-thre_2;
   end
  
%    all_contact_force_after=[all_contact_force_after contact_force_after];
   
contact_force_after(1)=predict_x*3;
contact_force_after(2)=predict_y*3;
contact_force_after(3)=predict_z*3;

 if i <20
     contact_force_after(1)=0;
     contact_force_after(2)=0;
     contact_force_after(3)=0;
 end
 


        v_cartesian_target=J_dx_dq*target_joint_velocity;
%         all_v_cartesian_target=[all_v_cartesian_target v_cartesian_target];
        
        
        
        v_cartesian = J_dx_dq*feedback_joint_velocity;
%         all_v_cartesian=[all_v_cartesian v_cartesian];
        
v_filtered1 = kalman1(v_cartesian(1));
v_filtered2 = kalman2(v_cartesian(2));
v_filtered3 = kalman3(v_cartesian(3));
  
v_filt=[v_filtered1 v_filtered2 v_filtered3].';
% new_v_filt=[new_v_filt v_filt];

        
feedback_joint_velocity_after=pinv(J_dx_dq)*v_filt;

%% SVD
% [U,S,V] = svd(J_dx_dq)











%         max=5;   % 8  8 5
%         k=max/50; 
%         if i <=1500 && i>1450
%             contact_force_after=[0 0 k*(i-1450)].';
%         elseif i >1500 && i<=1900
%             contact_force_after=[0 0 max].';
%         elseif i >1900 && i<=1950
%             contact_force_after=[0 0 max-k*(i-1900)].';
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

%% 3

    a_d=(J_dx_dq*target_joint_velocity_next-J_dx_dq*target_joint_velocity)/dt;
    
       xe=-target_end_effector_p+end_effector_p+J_dx_dq*target_joint_velocity*dt; %3 1
       xde=-J_dx_dq*target_joint_velocity+v_filt+a_d*dt;
        
      x_t1dd=H_inv*(contact_force_after-k_cartesian*xe-b_cartesian*xde);
%         equal_F=(-k_cartesian*xe-b_cartesian*xde);
%         all_f_attractor=[all_f_attractor equal_F];
     xdetjia=-J_dx_dq*target_joint_velocity+v_filt+x_t1dd*dt;
     
     xetjia=-target_end_effector_p+end_effector_p+xdetjia*dt;
%      all_xe=[all_xe xetjia];

        
        goal=(target_end_effector_p_next+xetjia).'; %3 1-> 13
        
%         goal=target_end_effector_p_next.';
%         intergal_goal=[intergal_goal; goal;];
        
        

  %% PD
  sum_e=sum_e+end_effector_p-goal.';
        output=P_coe*(end_effector_p-goal.')...
            +I_coe*(sum_e)...
            +D_coe*(v_filt-v_cartesian_target);
            
 %3 1
output=-output+goal.';
% all_output=[all_output output];
%%        
    init_theta1=180;
    init_theta2=0;
    
%     xd=goal(1);
%     yd=output(2);
%     zd=output(3);
    
    xd=goal(1);
    yd=goal(2);
    zd=goal(3);
    
    [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
    count_no=0;
            while isempty(All_theta)
                    if yd<0
                        init_theta2=init_theta2+1;
%                         init_theta1=180-count_no;
                    elseif yd>0
                        init_theta2=init_theta2+1;
%                         init_theta1=180+count_no;
                    else
                        init_theta2=init_theta2+1;
%                         init_theta1=180+count_no;
                    end
                count_no=count_no+1;
                [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
            end
    [hang,lie]=size(All_theta);
    temp=(target_joint_position.')*180/pi;
    tott=1000;
    for each_lie =1:lie
        this=All_theta(:,each_lie);
        this(7)=0;
        now=sum(abs(this-temp));
        
        if now < tott
            tott=now;
            which=each_lie;
        end
        
    end
    to_add=All_theta(:,which);
    to_add(7)=0;
    to_add=to_add*pi/180;
%     theta_points_final=[theta_points_final to_add];    
 %% 

        goal2 = satq(qmin,qmax,to_add.');
%         all_goal_theta=[all_goal_theta; goal2;];
        t = t+dt;

         [ pose_end, nsparam_end, rconf_end, jout_end ] = ForwardKinematics( goal2, robot_type );
        end_effector_p_end = pose_end(1:3,4);
%         pos_x=[pos_x end_effector_p_end];


    this_theta_matrix=target_joint_position;
    this_theta=num2cell(this_theta_matrix);

    this_zukang=num2cell(goal2);

%     if i < 60
        my_t=iiwa.sendJointsPositionsExTorque(this_zukang);
%     elseif flagg==3
%         my_t=iiwa.sendJointsPositionsExTorque(this_theta);
%     else 
%         my_t=iiwa.sendJointsPositionsExTorque(this_zukang);
%     end
    
%     all_jtor=[all_jtor; my_t;];

    
%     iiwa.realTime_stopImpedanceJoints()
end



tstart=t0;
tend=time;
rate=counter/(tend-tstart);
%% Stop the direct servo motion


% iiwa.realTime_stopVelControlJoints();


fprintf('\nTotal execution time is %f: \n',tend-t0 );
fprintf('\nThe rate of joint nagles update per second is: \n');
disp(rate);
fprintf('\n')
pause(2);
%% turn off server
% iiwa.realTime_stopImpedanceJoints()
iiwa.realTime_stopDirectServoJoints();
iiwa.net_turnOffServer()

disp('Direct servo motion completed successfully')
warning('on')
% figure(9);plot(intergal_goal(:,1)); hold on; plot(all_end_effector_p(1,:)); hold on;plot(all_output(1,:));
% legend('goal','real','PD')
% figure(10);plot(intergal_goal(:,2)); hold on; plot(all_end_effector_p(2,:));hold on;plot(all_output(2,:));
% legend('goal','real','PD')
% figure(11);plot(intergal_goal(:,3)); hold on; plot(all_end_effector_p(3,:));hold on;plot(all_output(3,:));
% legend('goal','real','PD')

% figure(12);plot(theta_points_final(2,:)); hold on; plot(all_feedback_joint_position(:,2))
% legend('goal','real')
%          clust = parcluster('local');
%          job1 = createJob(clust); %开启一个job
%          disp('saving------');
%          temp = all_v_cartesian.';
%          createTask(job1,@mytxt,1,{temp});%再给job1分配一个‘mytxt’的task
%          submit(job1);

