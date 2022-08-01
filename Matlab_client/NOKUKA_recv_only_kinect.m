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


clear
clc
close all
warning off all

addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main')  


draw_points=200;FLAG=0;predict_y=[];
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
all_delta_jian_y=[];all_delta_jian_x=[];all_delta_jian_z=[];
all_delta_zhou_y=[];all_delta_zhou_x=[];all_delta_zhou_z=[];
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
T_tot=20;all_predict_y=[];all_predict_x=[];all_predict_z=[];
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(theta_points,2)-1):T_tot;all_new_EMG=[];

[theta,theta_dot,theta_dotdot,pp] = cubicpolytraj(theta_points,tpts,tvec,...
                'VelocityBoundaryCondition', theta_d_des);

%% Start direct servo in joint space       
all_output=[];

w=1.5; % motion constants, frequency rad/sec
A=pi/6; % motion constants, amplitude of motion
counter=0;
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];v_filt=[];new_v_filt=[];
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
pause(3);all_new_EMG=[];all_new_EMG2=[];

%%  please prepare for starting
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
     current_combine=[];predict_x=0;predict_y=0;predict_z=0;
for i=1:size(theta_dot,2)-1

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
%     
% if length(find(recv2=='0'))==length(recv2)
%     recv2=[];
% end



        
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


%% 坐标转换
%         new_EMG=[current_combine];new_EMG2=[current_combine];
%         this_sliding=current_combine;
%                 delta_z=this_sliding(:,12)-this_sliding(:,3);
%                 delta_x=this_sliding(:,10)-this_sliding(:,1);
%                 delta_theta=atan(delta_z./delta_x);
%                 all_delta_theta=[all_delta_theta; delta_theta*180/pi;];
%      for k = 1:3
%         if k==1
%             who_x=7;
%             who_z=9;
%             who_y=8;
%         elseif k==2
%             who_x=4;
%             who_z=6;
%             who_y=5;
%         else
%             who_x=1;
%             who_z=3;
%             who_y=2;            
%         end
%                 rela_wrist_sh_x=this_sliding(:,who_x);
%                 rela_wrist_sh_z=this_sliding(:,who_z);
%               
%                     rela_wan_y=this_sliding(:,who_y);
% 
%                     output_this_silde_x=[];output_this_silde_z=[];
%                     for each_in_silde = 1:size(this_sliding,1)
%                         each_rela_jian_z=rela_wrist_sh_z(each_in_silde);
%                         each_rela_jian_x=rela_wrist_sh_x(each_in_silde);
%                         each_theta=delta_theta(each_in_silde);
% %                         each_theta = pi/2-mean(delta_theta);
%                         after_x=cos(each_theta)*each_rela_jian_x+sin(each_theta)*each_rela_jian_z;
%                         after_z=cos(each_theta)*each_rela_jian_z-sin(each_theta)*each_rela_jian_x;
%                         output_this_silde_x=[output_this_silde_x; after_x;];
%                         output_this_silde_z=[output_this_silde_z; after_z;];
%                     end
%                     new_EMG2=[ new_EMG2  output_this_silde_x rela_wan_y output_this_silde_z];    
%      end
%      new_EMG=[current_combine new_EMG2(:,13:15)-new_EMG2(:,19:21) new_EMG2(:,16:18)-new_EMG2(:,19:21)];
%      
%      all_new_EMG2=[all_new_EMG2; new_EMG2;];
    all_new_EMG=[all_new_EMG; current_combine;];
  
                    rela_wan_z=current_combine(:,9)-current_combine(:,6);
                    rela_wan_y=current_combine(:,8)-current_combine(:,5);
                    rela_wan_x=current_combine(:,7)-current_combine(:,4);

                    rela_zhou_z=current_combine(:,6);
                    rela_zhou_y=current_combine(:,5);
                    rela_zhou_x=current_combine(:,4);                    
                    
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
                    
                    
                    delta_jian_x=-sum(current_combine(1:1,1))+sum(current_combine(end:end,1));
                    delta_jian_y=-sum(current_combine(1:1,2))+sum(current_combine(end:end,2));
                    delta_jian_z=-sum(current_combine(1:1,3))+sum(current_combine(end:end,3));
                    
%                     if delta_wan_x > 0.04 || delta_jian_x > 0.04 || delta_zhou_x > 0.04
%                         predict_x=predict_x+1;
%                     elseif delta_wan_x < -0.04 || delta_jian_x < -0.04 || delta_zhou_x < -0.04
%                         predict_x=predict_x-1;
%                     else
%                         predict_x_count=1;
%                     end

 %%  threhold_velocity_method
%         threhold_velocity=0.05;            
%                      if delta_wan_x > threhold_velocity || delta_jian_x > threhold_velocity || delta_zhou_x > threhold_velocity
%                         predict_x=predict_x+1;
%                     elseif delta_wan_x < -threhold_velocity || delta_jian_x < -threhold_velocity || delta_zhou_x < -threhold_velocity
%                         predict_x=predict_x-1;
%                     else
%                         predict_x_count=1;
%                      end
%                     
%                                         
%                     if predict_x > 10
%                         predict_x=predict_x-1;
%                     elseif predict_x <-10
%                         predict_x=predict_x+1;                
%                     elseif predict_x < 0
%                         predict_x=predict_x+predict_x_count;
%                     elseif predict_x > 0
%                         predict_x=predict_x-predict_x_count;
%                     end
%                     
%                     
%                     all_predict_x=[all_predict_x predict_x];
%                     
%                     if delta_wan_y > threhold_velocity || delta_zhou_y > threhold_velocity  % only motion of wrist related to elbow
%                         predict_y=predict_y+1;
%                     elseif delta_wan_y < -threhold_velocity || delta_zhou_y < -threhold_velocity
%                         predict_y=predict_y-1;
%                     else
%                         predict_y_count=1;
%                     end
%                     
%                                         
%                     if predict_y > 10
%                         predict_y=predict_y-1;
%                     elseif predict_y <-10
%                         predict_y=predict_y+1;
%                     elseif predict_y < 0
%                         predict_y=predict_y+predict_y_count;
%                     elseif predict_z > 0
%                         predict_y=predict_y-predict_y_count;
%                     end
%                     
%                     all_predict_y=[all_predict_y predict_y];
%                     
%                     if delta_zhou_z > threhold_velocity*0.4 || delta_jian_z > threhold_velocity*0.4
%                         predict_z=predict_z+1;
%                     elseif delta_zhou_z < -threhold_velocity*0.4 || delta_jian_z < -threhold_velocity*0.4
%                         predict_z=predict_z-1;
%                     else
%                         predict_z_count=1;
%                     end
%                                
%                     if predict_z > 10
%                         predict_z=predict_z-1;
%                     elseif predict_z <-10
%                         predict_z=predict_z+1;
%                     elseif predict_z < 0
%                         predict_z=predict_z+predict_z_count;
%                     elseif predict_z > 0
%                         predict_z=predict_z-predict_z_count;
%                     end
% 
%                     all_predict_z=[all_predict_z predict_z];  
%%
% [A1,PS]=mapminmax([-5 0 1])


%%
here_wan=[delta_wan_x; delta_wan_y; delta_wan_z;];
maxx_wan=max(abs(here_wan));
new_here_wan=here_wan/maxx_wan;
if maxx_wan < 0.004
    new_here_wan=[0 0 0];
end
here_zhou=[delta_zhou_x; delta_zhou_y; delta_zhou_z;];
maxx_zhou=max(abs(here_zhou));
new_here_zhou=here_zhou/maxx_zhou;
if maxx_zhou < 0.004
    new_here_zhou=[0 0 0];
end
here_jian=[delta_jian_x; delta_jian_y; delta_jian_z;];
maxx_jian=max(abs(here_jian));
new_here_jian=here_jian/maxx_jian;
if maxx_jian < 0.004
    new_here_jian=[0 0 0];
end




    now_predict_y=new_here_wan(2)+new_here_zhou(2);
    now_predict_z=new_here_wan(3)+new_here_jian(3);
    now_predict_x=new_here_zhou(1)+new_here_jian(1);
    if abs(now_predict_x) < 0.7
        predict_x_count=1;
    end
     if abs(now_predict_z) < 0.7
        predict_z_count=1;
    end   
     if abs(now_predict_y) < 0.7
        predict_y_count=1;
     end
    
    predict_z=predict_z+now_predict_z;
    predict_x=predict_x+now_predict_x;
    predict_y=predict_y+now_predict_y;
                    if predict_z > 10
                        predict_z=predict_z-abs(now_predict_z);
                    elseif predict_z <-10
                        predict_z=predict_z+abs(now_predict_z);
                    elseif predict_z < 0
                        predict_z=predict_z+predict_z_count;
                    elseif predict_z > 0
                        predict_z=predict_z-predict_z_count;
                    end
                    if predict_x > 10
                        predict_x=predict_x-abs(now_predict_x);
                    elseif predict_x <-10
                        predict_x=predict_x+abs(now_predict_x);
                    elseif predict_x < 0
                        predict_x=predict_x+predict_x_count;
                    elseif predict_x > 0
                        predict_x=predict_x-predict_x_count;
                    end
                    if predict_y > 10
                        predict_y=predict_y-abs(now_predict_y);
                    elseif predict_y <-10
                        predict_y=predict_y+abs(now_predict_y);
                    elseif predict_y < 0
                        predict_y=predict_y+predict_y_count;
                    elseif predict_y > 0
                        predict_y=predict_y-predict_y_count;
                    end
    

all_predict_x=[all_predict_x predict_x];  
all_predict_y=[all_predict_y predict_y];  
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

pause(0.01);
end

                    all=[all_delta_wan_x; all_delta_wan_y; all_delta_wan_z;
                       all_delta_zhou_x; all_delta_zhou_y; all_delta_zhou_z;
                       all_delta_jian_x; all_delta_jian_y; all_delta_jian_z;].';

                   
figure(99); plot(abs(all_delta_wan_z)./abs(all_delta_wan_x));hold on;
plot(abs(all_delta_wan_z)./abs(all_delta_wan_y));hold on;
plot(abs(all_delta_wan_z)./abs(all_delta_wan_z));hold on;
legend('x','y','z')
ylim([-10,10])

jj=abs([all_delta_zhou_x; all_delta_zhou_y; all_delta_zhou_z;]);
[r,c,v]=find(jj==max(jj));
figure(78)
plot(r);hold on; plot(all_predict_z);

%   figure(56); all_dx=[];all_dy=[];               
% for q = 1:2990-6
%     y1=all_sum_wan_y(q);
%     y2=all_sum_wan_y(q+6);
%     x1=all_sum_wan_x(q);
%     x2=all_sum_wan_x(q+6);
%     d_x=x2-x1;
%     d_y=y2-y1;
%     all_dx=[all_dx d_x];
%     all_dy=[all_dy d_y];
% end
% plot(all_dx); hold on; plot(all_dy);          
                   
                   
figure;plot(all_new_EMG(:,16:end),'DisplayName','all_new_EMG(:,16:end)')
figure;plot(all_new_EMG(:,13:15),'DisplayName','all_new_EMG(:,13:15)')
figure;plot(all_new_EMG(:,13:15)-all_new_EMG(:,16:end))

                   
                   
                   
                   