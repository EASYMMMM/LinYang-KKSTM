%% Example of using KST class for interfacing with KUKA iiwa robots
% 有几个问题啊，z轴运动，斜切角度，运行周期，转向后切过了，蜜汁抽出

close all;clear;clc;
warning('off')
TIMEE=[];
%% 传数
t_server_self=tcpip('0.0.0.0',30000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
t_server_self.InputBuffersize=100000;
disp(['未打开！',datestr(now)])
fopen(t_server_self);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])
%%
IP_remote_IMU = "192.168.11.1"; 
port_remote_IMU = 5000;
IP_local_IMU = "192.168.11.2"; 
port_local_IMU = 5000;
Role_IMU = 'client';
t_server_IMU = tcpip(IP_remote_IMU,port_remote_IMU,...
                'NetworkRole',Role_IMU,...
                'LocalPort',port_local_IMU,...
                'TimeOut',20,...
                'InputBufferSize',8192);

t_server_IMU.InputBuffersize=100000;

disp(['未打开！',datestr(now)])
fopen(t_server_IMU);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])

data_all_IMU=[];count_right_IMU=0;level=1;

%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main-raw')  
addpath('C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client_rawing\EMG_IMU')
%% Initalize the coefficient of LASSO and judge 
file='ylcoef_0219.mat';
coef_4=load(file).coef_4;
coef_2=load(file).coef_2;
fitinfo2=load(file).fitinfo2;
fitinfo4=load(file).fitinfo4;
coef_3=load(file).coef_3;
fitinfo3=load(file).fitinfo3;
Mdl_1=load(file).Mdl_1;
Mdl=load(file).Mdl;
%% Start a connection with the server
flag=iiwa.net_establishConnection();
if flag==0
  return;
end
pause(1);
disp('Moving first joint of the robot using a sinusoidal function')
    

%% Go to initial position
relVel=0.15; % the relative velocity
% theta_points=[-0.401300000000000;0.953900000000000;0;-1.04900000000000;0;1.13740000000000;0].';
theta_points=[0,0.772629915332932,0,-1.26703950381033,0,1.10192323444653,0];


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
kalmanF = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalmanV = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
%% Start direct servo in joint space       
w=1.5; % motion constants, frequency rad/sec
A=pi/6; % motion constants, amplitude of motion
counter=0;
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];v_filt=[];new_v_filt=[];all_acc=[];all_init_direction_theory=[];init_direction_theory=[0.65 0 0.2].';
all_final_new_target_velocity_7=[];all_final_new_target_position_7=[];
STOP=0;
%% Start direct servo in joint space       
iiwa.realTime_startVelControlJoints();all_output=[];
% iiwa.realTime_startImpedanceJoints(0.6,0,0,0.1,3500,250,10);

% iiwa.realTime_startVelControlJoints();

w=1.5; % motion constants, frequency rad/sec
A=pi/6; % motion constants, amplitude of motion
counter=0;
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];v_filt=[];new_v_filt=[];all_this_y_v=[];come_in=[];
%% Initiate PIDDDDDDDDDDDDDDDDDDDDDDDDDD variables
k=[0.9089 0.8639 0.324];
k1=1;
k2=1;
k3=1;

%     k_cartesian = diag([100,100,100]*1*1)*15;
%     b_cartesian = diag([100,100,100]*14*0.707*45/1000*2.5*3);   
%     H_inv = diag([1 1 1]/10*0.8/3);
%     w_n=(k_cartesian.*H_inv)^0.5;
%     w_n=w_n(1,1)
%     zeta=b_cartesian.*H_inv/2./w_n;
%     zeta=zeta(1,1)

%     k_cartesian = diag([100*k1,100*k2,100*k3]*1*1);
%     b_cartesian = diag([100,100,100]*14*0.707*45/1000);   
%     H_inv = diag([1*k1 1*2 1*k3]/10);
%     k_cartesian = diag([100,100,100]*1*1)*1.3*5;
%     b_cartesian = diag([100,100,100]*14*0.707*45/1000*0.7*10/2*2.5);   
%     H_inv = diag([1 1 1]/10/5*2);
all_target_joint_position_3_ori=[];lock=0;


    k_cartesian = diag([100,100,100]*1*1)*1.3*5/2;
    b_cartesian = diag([100,100,100]*14*0.707*45/1000*0.7*5*1.4/2*1.4);   
    H_inv = diag([1 1 1]/10/5*3*2/2);
    
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
             new_f=[];new_v=[];new_torque=[];all_v_7dof=[];
    all_feedback_joint_position=[];
%% Control loop   
all_v_cartesian_target=[];all_v_cartesian=[];alldt=[];all_contact_force_after=[];
all_target_joint_position_e=[];
All_v=[];EX_force_ori=[];all_F_contact=[];my_t=[{0} {0} {0} {0} {0} {0} {0}];all_f_attractor=[];all_end_effector_p=[];
robot_type=1;  all_jpos=[]; all_jtor=[];my_torque=[0 0 0 0 0 0 0];F_contact=0;all_joint_pos=[];all_target_end_effector_p=[];
all_dt_real=[];
intergal_goal=[];adjust=[];

init_direction=[0.65; 0; 0.2];
accum_dt=3.5;all_xe=[]; all_xde=[];

feedback_joint_velocity_after=zeros(7,1);all_output=[];last_space=[{0} {0} {0} {0} {0} {0} {0}];
init_pos=[0,0.772629915332932,0,-1.26703950381033,0,1.10192323444653,0].';
count_loop=0;
rate_xdetjia=[];rate_target=[];
target_joint_position_next=[0,0.772629915332932,0,-1.26703950381033,0,1.10192323444653,0];
feedback_joint_position=[0,0.772629915332932,0,-1.26703950381033,0,1.10192323444653,0];
target_joint_position=target_joint_position_next;all_this_point_after=[];
FINISH=0;round=0; compared_p=[];compared_v=[];all_target_joint_position=[];all_real_joint_position=[];
angle_7=0;up=0;
last_state=8;all_real_point=[];MDZZ=[];
target_joint_velocity_next=[0 0 0 0 0 0 0].';all_this_point=[];last_p=target_joint_position_next;all_xetjia=[];v_filt=[0 0 0].';
tic;
Ts=0.010;count2=0;angle_comb=0;
dt_real=Ts;
data_all = [];all_refer=[];all_BIGTIME=[];all_pre_z=[];
    count_self = 0;NEVER=0;DRAW=0;ori_this_point_after=[];

count_y=0;
%代表那个大循环   
    go=0;
    all_y_pro=[0];all_y=[];all_y_pro_fil=[0];
all_a=[]; all_b=[]; all_y=[];predict_test_y_KKK=[];FUCKact=[];
%%  please prepare for starting
while  go  == 0

tic
 
     %%       
    if  t_server_self.BytesAvailable>0
     data_recv_self = fread(t_server_self,t_server_self.BytesAvailable/8,'double');%    disp(size(data_recv));
    count_self = count_self + 1;
    which_head=find(88887<=data_recv_self);
    which_head2=which_head(end);
    this_frame=data_recv_self(which_head2:end);
    if length(this_frame)~=3
        go = 1;
    end
    end        
%%

            if  t_server_IMU.BytesAvailable>0
                data_recv_IMU = fread(t_server_IMU,t_server_IMU.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
                
                which_head_IMU=find(88887<=data_recv_IMU);
                if ~isempty(which_head_IMU)
                    which_head2_IMU=which_head_IMU(end);
                    this_frame_IMU=data_recv_IMU(which_head2_IMU:end);
                    if length(this_frame_IMU) == 76
       
                  
                        this1=this_frame_IMU(1+1:25+1,:).';
                    this2=this_frame_IMU(26+1:50+1,:).';
                    this3=this_frame_IMU(51+1:75+1,:).';              
                        R1 = rotate_matrix(this1);
                        R2 = rotate_matrix(this2);
                        R3 = rotate_matrix(this3);
                        R31 = R1.'*R3;
                        R23 = R3.'*R2;
                        [b,a,y] = inverse_angle(R31);
                        all_a=[all_a a];
                        all_b=[all_b b];
                        all_y=[all_y y];    
                    end
                end
            end
 %%           
       
     pause(0.01);

toc
end

y0=all_y(end);
judge_EMG=0.13;  SONGSHOU=0;
%% CONTROAL LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP
disp(' TIME TO GOOOOO')    
output=[];predict_F=[];count_up=0;laststate=8;BIGTIME=2;ENDD=0;
while FINISH == 0
    if ENDD == 1
        break
    end
    
    if STOP >= 10
        if round > mark_round+2
            FINISH=1;
            break
        end
    end
    STOP=0;
    
           round=round+1;
          contact_force_after=[0; 0; 0;];v_filt=[0;0;0;];delete_point=1;accu_point=1;diminish=0;
          DOWN=0;UP=0;
          
          if count_up>15 && NEVER == 0 && laststate <=6
              up=1
              NEVER=1
          else
              up=0
          end
          
          
          
          
          
          
          count_up=0;
            this_p_7=init_pos;
            [ pose, nsparam, rconf, jout ] = ForwardKinematics( this_p_7, robot_type );
            now_pos_3=pose(1:3,4)           
            [total_act way_points which_state_now myspace]= RL2m3m3(now_pos_3,angle_7,up,lock,last_space,laststate,round);
            laststate=which_state_now
            lock
            last_space=myspace;
            if which_state_now == 4 || which_state_now == 1
                lock=1
                angle_comb=atan((way_points(2,1)-way_points(2,2))/(way_points(1,1)-way_points(1,2)))    
                rotate_matrix_acc=[ [cos(angle_comb), -sin(angle_comb) 0]; [sin(angle_comb), cos(angle_comb) 0]; [0 0 1];];
            end
            if total_act(1)-total_act(2) == 6
                DOWN=1
            end
            if total_act(1)-total_act(2) == -6
                UP=1
            end            
            
            way_points
            total_act 

                last_state=which_state_now;

            
            [useless, len_way_points]=size(way_points);
           theta_points= feedback_joint_position.';
%            theta_points=this_p_7;
%            theta_points=target_joint_position.';
            theta_points=theta_points*180/pi;
            
%   [new_way_points]= simplifywaypoints(way_points,total_act)
  [hang,lie]=size(way_points);
  
  
for i_inv = 2 : lie
    init_theta1=180;
    init_theta2=0;
    xd=way_points(1,i_inv);
    yd=way_points(2,i_inv);
    
        if now_pos_3(3)<0.11 && up == 0
            zd=now_pos_3(3)-0.001;
        else
            zd=way_points(3,i_inv);
        end
    
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
    
%     All_theta=All_theta+[180
    [hang,lie]=size(All_theta);
    temp=theta_points(:,end);
    tott=1000;
    delta_matrix=All_theta-temp;
    for each_lie =1:lie
        now=sum(abs(All_theta(:,each_lie)-temp));
%          now=abs(sum(All_theta(:,each_lie)-temp))
%         now=abs(sum(All_theta(1:end-1,each_lie)-temp(1:end-1,:)))
        if now < tott;
            tott=now;
            which=each_lie;
        end
        
    end
    to_add=All_theta(:,which);
    to_add(7)=0;
    theta_points=[theta_points to_add];
end
theta_points=theta_points.*pi/180;



theta_d_des=[];
for z =1:len_way_points
    theta_d_des=[theta_d_des [0;0;0;0;0;0;0]];
end

if up == 1 || DOWN == 1
    BIGTIME=3;
end


T_tot=(size(theta_points,2)-1)*BIGTIME
BIGTIME=floor(accum_dt);

if BIGTIME <=1
    BIGTIME=1;
end
if BIGTIME >=3
    BIGTIME=3;
end

all_BIGTIME=[all_BIGTIME BIGTIME];
accum_dt=0;
if T_tot == 0
    disp('arrive')
    break
end
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(theta_points,2)-1):T_tot;

[theta,theta_dot,theta_dotdot,pp] = cubicpolytraj(theta_points,tpts,tvec,...
                'VelocityBoundaryCondition', theta_d_des);

figure; plot(cao(2,:)); hold on; plot(theta_dot(2,2:end))      
            
            

refer_pos=[];
for q=1:size(theta_dot,2)
         [ poseo, nsparam, rconf, jout ] = ForwardKinematics( theta(:,q).', robot_type );
        end_effector_po = poseo(1:3,4);
        refer_pos=[refer_pos end_effector_po];
end
all_refer=[all_refer refer_pos];

over=0;
for i_theta=1:1000
%%    control  
    i_theta;
    if i_theta == 1
        start = toc
    end
    accum_dt=accum_dt+dt_real;
    all_dt_real=[all_dt_real dt_real];
    dt=dt_real;

    count_loop=count_loop+1;

        this_point=ceil(accum_dt/T_tot*size(theta_dot,2));
        direction=which_refer_direction(total_act);
        [y,iy]=min(abs(refer_pos(direction,:)-init_direction(direction)));
        
         real_point=iy;

         
        if real_point >= size(theta_dot,2)
            real_point = size(theta_dot,2);
        end 

    
    if this_point >= length(tvec)/(size(theta_points,2)-1)*3
        this_point = size(theta_dot,2);
        this_point
        break
    end
  [v_direction]= which_refer_v(total_act);
    
  %% stay at one place  
%  if v_direction*contact_force_after(1)<0
 if abs(contact_force_after(1))>20
     diminish=1;
     accu_point=real_point;
     delete_point=this_point;
     MDZZ=[MDZZ 1];
 else
     MDZZ=[MDZZ 0];
 end
 %% over代表已经越过了轨迹终点   
 if over == 0
     this_point_after=this_point-diminish*(delete_point-accu_point);
 else
     this_point_after=size(theta_dot,2);
 end
    ori_this_point_after=[ori_this_point_after this_point_after];
    
    
    FUCKact=[FUCKact total_act(1)-total_act(2)];
    if this_point_after < this_point
        this_point_after = this_point;
    end
    
    
    if this_point_after >= size(theta_dot,2)
        over=1;
        if total_act(1)-total_act(2) == -3 && output(end) == 2
                this_point_after = size(theta_dot,2);
                come_in=[ come_in 1];
        elseif total_act(1)-total_act(2) == 3 && output(end) == 3
                this_point_after = size(theta_dot,2);
                come_in=[ come_in 2];
        else
            come_in=[come_in 4];
            break
        end
    else
        come_in=[come_in -1];
    end
    
    all_real_point=[all_real_point real_point]; 
    all_this_point=[all_this_point this_point];
    all_this_point_after=[all_this_point_after this_point_after];
    
    if over == 0
        target_joint_position=target_joint_position_next;
        target_joint_position_next=theta(:,this_point_after+1).';
        target_joint_velocity=target_joint_velocity_next;
        target_joint_velocity_next=theta_dot(:,this_point_after+1);
    else
        target_joint_position=target_joint_position_next;
        target_joint_position_next=theta(:,this_point_after).';
        target_joint_velocity=target_joint_velocity_next;
        target_joint_velocity_next=theta_dot(:,this_point_after);
    end
    
    
    
    
    
    
    all_target_joint_position=[all_target_joint_position; target_joint_position;];

    
%% caculate outer force

    [ pose_c, nsparam_c, rconf_c, jout_c ] = ForwardKinematics( target_joint_position.', robot_type );
    target_end_effector_p = pose_c(1:3,4);
    all_target_end_effector_p=[all_target_end_effector_p target_end_effector_p];  %-----------
    
    %% ========Adj

% time update
        [Jac_this,A_mat_products] = Jacobian(target_joint_position,robot_type);
        J_dx_dq_this = Jac_this(1:3,:);
    [ pose_c2, nsparam_c, rconf_c, jout_c ] = ForwardKinematics( target_joint_position_next.', robot_type );
    target_end_effector_p_next = pose_c2(1:3,4);
    all_target_end_effector_p_next=[all_target_end_effector_p_next target_end_effector_p_next];
    
    this_p=iiwa.getJointsPos();
    this_p=cell2mat(this_p);
    feedback_joint_position=this_p;
    angle_7=feedback_joint_position(end);
    all_jpos=[all_jpos; this_p;];  
        [ pose_e, nsparame, rconfe, joute ] = ForwardKinematics( feedback_joint_position, robot_type );
        end_effector_p = pose_e(1:3,4);
        all_end_effector_p=[all_end_effector_p end_effector_p];
        
        
    
   
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




        
        %% judge stay at one place 
       
        
        end_effector_r = pose_e(1:3,1:3);


        f_bias=[4.3 0 17.75].';
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
    
 if i_theta <10
     contact_force_after(3)=0;
     contact_force_after(1)=0;
     contact_force_after(2)=0;
 end
  %%
 if end_effector_p(3) < 0.12 && up == 0
     contact_force_after(3)=contact_force_after(3)/3;


    [ pose_c3, nsparam_c, rconf_c, jout_c ] = ForwardKinematics( feedback_joint_position.', robot_type );
    target_joint_position_3_ori = pose_c3(1:3,4);
    
    target_joint_velocity_3_ori=J_dx_dq*feedback_joint_velocity_after;

%     target_joint_position_3_ori(3)=target_joint_position_3_ori(3)-level*0.0001/4;
%     target_joint_velocity_3_ori(3)=target_joint_velocity_3_ori(3)-level*0.00001/4;
     final_new_target_velocity_7=pinv(J_dx_dq)*target_joint_velocity_3_ori;
     
     
     threhold_friction=10;
     if abs(contact_force_after(1)) <= threhold_friction
         contact_force_after(1)=0;
     elseif contact_force_after(1) < -threhold_friction
        contact_force_after(1)=contact_force_after(1)+threhold_friction;
     elseif contact_force_after(1) > threhold_friction
         contact_force_after(1)=contact_force_after(1)-threhold_friction;
     end
     
      if abs(contact_force_after(2)) <= threhold_friction
         contact_force_after(2)=0;
     elseif contact_force_after(2) < -threhold_friction
        contact_force_after(2)=contact_force_after(2)+threhold_friction;
     elseif contact_force_after(2) > threhold_friction
         contact_force_after(2)=contact_force_after(2)-threhold_friction;
      end    
      
      if abs(contact_force_after(3)) <= threhold_friction
         contact_force_after(3)=0;
     elseif contact_force_after(3) < -threhold_friction
        contact_force_after(3)=contact_force_after(3)+threhold_friction;
     elseif contact_force_after(3) > threhold_friction
         contact_force_after(3)=contact_force_after(3)-threhold_friction;
      end 
      
     all_target_joint_position_3_ori=[all_target_joint_position_3_ori target_joint_position_3_ori];
     all_final_new_target_velocity_7=[all_final_new_target_velocity_7 final_new_target_velocity_7];
     
     target_end_effector_p=target_joint_position_3_ori;
     target_joint_velocity=final_new_target_velocity_7;
 end

    %% human's intension
            if  t_server_self.BytesAvailable>0
                    data_recv_self = fread(t_server_self,t_server_self.BytesAvailable/8,'double');%    disp(size(data_recv));
                    count_self = count_self + 1;
                    which_head_self=find(88887<=data_recv_self);
                    which_head2_self=which_head_self(end);
                    this_frame_self=data_recv_self(which_head2_self:end);
                    data_all(:,count_self) = this_frame_self;
            end
            
            if  t_server_IMU.BytesAvailable>0
                data_recv_IMU = fread(t_server_IMU,t_server_IMU.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
                
                which_head_IMU=find(88887<=data_recv_IMU);
                if ~isempty(which_head_IMU)
                    which_head2_IMU=which_head_IMU(end);
                    this_frame_IMU=data_recv_IMU(which_head2_IMU:end);
                    if length(this_frame_IMU) == 76
                        
                        count_right_IMU = count_right_IMU + 1;
                        data_all_IMU(:,count_right_IMU) = this_frame_IMU;
                        this1=this_frame_IMU(1+1:25+1,:).';
                    this2=this_frame_IMU(26+1:50+1,:).';
                    this3=this_frame_IMU(51+1:75+1,:).';              
                        R1 = rotate_matrix(this1);
                        R2 = rotate_matrix(this2);
                        R3 = rotate_matrix(this3);
                        R31 = R1.'*R3;
                        R23 = R3.'*R2;
                        [b,a,y] = inverse_angle(R31);
                        all_a=[all_a a];
                        all_b=[all_b b];
                        all_y=[all_y y];    
                    end
                end
            end
%% 计算角度与速度            
 
        
    next_y= y-y0;
    count_y=count_y+1;
        this_y = all_y_pro(count_y);
        
        if next_y<-320
            next_y=next_y+360;
        end
        append_y=next_y;
        if abs(next_y-this_y)>10
            append_y=this_y;
        end
        all_y_pro=[all_y_pro append_y];
        

        all_y_filtered = kalmanV(append_y.');
        all_y_pro_fil=[all_y_pro_fil all_y_filtered];
        this_y_v=all_y_pro_fil(end) - all_y_pro_fil(end-1);
        all_this_y_v=[all_this_y_v this_y_v];

        this_frame_EMG=this_frame_self(2:end-1);
        EMG1=this_frame_EMG(1);
        EMG_left=this_frame_EMG(2:end).';
        pre_z=EMG_left*coef_4+fitinfo4.Intercept;
        all_pre_z=[all_pre_z pre_z];
    if EMG1 >= 0.4
            if EMG1>1.5
                level=2;
            else
                level=1;
            end

%         figure(99)
%         plot(output); hold on; 
%         plot(all_end_effector_p(1,:)); hold on;
%         plot(predict_test_y_KKK); hold on; 
%         plot(all_contact_force_after(1,:));
    
        
           outclass = predict(Mdl,EMG_left);
           if outclass == 1 || outclass == 2
               if SONGSHOU == 0 || NEVER ==1
                    output=[output 2];
               elseif SONGSHOU == 1
                    output=[output 1];
                    count_up=count_up+1;
               end   
               
           elseif outclass == 3
                output=[output 3];    
           else
                output=[output 0]; 
           end
           
            if length(output) >=11
                how3=length(find(output(end-10:end)==3));
                how2=length(find(output(end-10:end)==2));
                if how3>how2
                    pre_y=(EMG_left*coef_3+fitinfo3.Intercept)*how3/10;
                else
                    pre_y=(EMG_left*coef_2+fitinfo2.Intercept)*how2/10;
                end
            else
                pre_y=0;
            end
           
            predict_F=[predict_F pre_y];
             F_filtered_int = kalmanF(pre_y);
             predict_test_y_KKK=[predict_test_y_KKK; F_filtered_int;];
            F_filtered_int=F_filtered_int/5;
 %%          ?? v ->10   
            
             if end_effector_p(3) < 0.1
                 if  abs(this_y_v) < 0.1
                     contact_force_after(1)=contact_force_after(1)+F_filtered_int*cos(angle_comb);
                     contact_force_after(2)=contact_force_after(2)+F_filtered_int*sin(angle_comb);
                 else
                     contact_force_after(1)=contact_force_after(1)+(-this_y_v/abs(this_y_v)*10+F_filtered_int)*cos(angle_comb);
                     contact_force_after(2)=contact_force_after(2)+(-this_y_v/abs(this_y_v)*10+F_filtered_int)*sin(angle_comb);
                 end
             end
             
    else
        level=0;
        output=[output -1]; 
        STOP=STOP+1;
        disp('EMG nonooo')
    end
    
    if STOP>=10
        disp('STOPP')
        if SONGSHOU == 0
            mark_round=round;
        end
        
        if SONGSHOU==1
            break
        end
        
        if output(end) ~= -1 && SONGSHOU == 0
            SONGSHOU=1
            STOP=0;
        end       
        
    end
    
    if count_up > 15 && up == 0 && NEVER == 0 && laststate <=6
        disp('i want to go up')
        break
    end

%%  
if up == 1 || DOWN == 1 || UP == 1
    disp('NO UP and Down')
    count_up =0;
    
end
% !!!!!!!!!!!直接锁死z轴, 力的修复
if which_state_now == 4 || which_state_now == 1
    new_F=rotate_matrix_acc.'*contact_force_after;
    new_F_pro=[new_F(1); 0; 0;];
    back_F=rotate_matrix_acc*new_F_pro;
    contact_force_after=back_F;
elseif which_state_now == 2 || which_state_now == 5
    contact_force_after(2) = 0;
end
contact_force_after(3)=3-level*3;
contact_force_after(3)=0;

   all_contact_force_after=[all_contact_force_after contact_force_after];
%        contact_force_after=[0 0 0].';


        v_cartesian_target=J_dx_dq*target_joint_velocity;
        all_v_cartesian_target=[all_v_cartesian_target v_cartesian_target];
   
        
        v_cartesian = J_dx_dq*feedback_joint_velocity_after;
        all_v_cartesian=[all_v_cartesian v_cartesian];

  
% v_filt=[v_filtered1 v_filtered2 v_filtered3].';
new_v_filt=[new_v_filt v_filt];


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
%% 7 DOF
        if end_effector_p(3) > 0.12
               k_7dof=100*1.3*2.5;
               d_7dof=100*14*0.707*45/1000*0.7*5*1.4/2*1.4;
               h_7dof=1/10/5*3*2;
               Tor7=my_torque(end)*40;
               a_7dof=h_7dof*(Tor7-k_7dof*feedback_joint_position(end)-d_7dof*feedback_joint_velocity_after(end));
               v_7dof=feedback_joint_velocity_after(end)+a_7dof*dt;
               feedback_joint_velocity_after(end)=v_7dof;         
        end 
     %% SAFE
    future_pos_7=this_p.'+target_joint_velocity_next*dt;
    for each_joint = 1:7
        if (future_pos_7(each_joint) <= qmin(each_joint)) || (future_pos_7(each_joint) >= qmax(each_joint))
            disp('ERROR ! range is out of limit-1')
            ENDD=1;
            feedback_joint_velocity_after(each_joint)=0;
            break
        end
    end
    future_pos_3=end_effector_p+J_dx_dq*target_joint_velocity_next*dt;
    if future_pos_3(3)<0.05
        disp('ERROR ! range is out of limit-2')
        ENDD=1;
        break
    end

    all_feedback_joint_velocity_after=[all_feedback_joint_velocity_after feedback_joint_velocity_after];

    this_theta_matrix=target_joint_velocity_next.';
    this_theta=num2cell(this_theta_matrix);

    this_zukang=num2cell(feedback_joint_velocity_after);


        my_t=iiwa.sendJointsVelocitiesExTorques(this_zukang);


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


end

% figure(59);
% plot(new_f(3,:)); hold on;
% plot(all_contact_force_after(3,:)); hold on;
% plot(all_F_contact(3,:)); hold on; 
% plot(all_acc(3,:)*100); hold on; 
% plot(new_v_filt(3,:)*300); hold on;
% plot(all_init_direction_theory(3,:)*100-20); hold on; 
% plot(all_end_effector_p(3,:)*100-20);
% legend('KKKK','filter force','Force','acceleration','velocity','theortic position','real position')


% plot(all_contact_force_after(1,:)); hold on;plot(new_f(1,:)); hold on; plot(all_end_effector_p(1,:)*100-65);hold on; plot(new_v_filt(1,:)*100); 
figure(60)
plot(all_contact_force_after(3,:)); hold on;
plot(-all_f_attractor(end,:))
legend('filter force','following force')


%% turn off server
% iiwa.realTime_stopImpedanceJoints()
iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer();
pause(15)


fwrite(t_server_IMU,[88888.888,7654321],'double');%写入数字数据，每次发送360个double
fwrite(t_server_self,[88888.888,7654321],'double');%写入数字数据，每次发送360个double
fclose(t_server_self);
fclose(t_server_IMU);



disp('Direct servo motion completed successfully')
warning('on')
all_end_effector_p;
save('20220219M2_kasi.mat')

% load('20220127M2_diaole.mat')
% come_in(find(come_in == 0)) =4;
% figure(55)
% plot(all_end_effector_p(1,:)*3); hold on; 
% plot(all_this_point_after/100);  hold on; 
% plot(output); hold on;
% plot(FUCKact); hold on;
% plot(come_in)



%% see
result=[]; count_others=1;
for look = 1:length(come_in)
    if come_in(look) == 4
        result=[result; [4 4 ...
            4 FUCKact(look) come_in(look)];];        
    else
        
        result=[result; [all_end_effector_p(1,count_others)*3 all_this_point_after(count_others)/100 ...
            output(count_others) FUCKact(look) come_in(look)];];
        count_others=count_others+1;
    end
end 
figure(8);plot(result)
legend('real position','all_this_point_after','intent','act','comein')
    
           all_real_point=[all_real_point real_point]; 
    all_this_point=[all_this_point this_point];
    all_this_point_after=[all_this_point_after this_point_after];
    
figure(9)
plot(all_real_point); hold on; 
plot(all_this_point); hold on; 
% plot(ori_this_point_after); hold on; 
plot(all_this_point_after); 
legend('all_real_point','all_this_point','all_this_point_after')

check_maxi_axis=rotate_matrix_acc.'*all_target_end_effector_p;
F_maxi_axis=rotate_matrix_acc.'*all_contact_force_after;


% 分析z轴
figure(87)
plot(all_init_direction_theory(3,:)); hold on;
plot(all_target_end_effector_p_next(3,:)); hold on;
plot(all_end_effector_p(3,:))
legend('计算的理论','target','real')

%   figure(56)
% for q = 1:size(all_end_effector_p,2)
%     plot([all_end_effector_p(1,q), all_end_effector_p(2,q)]); hold on;
% end

    %
x=all_end_effector_p(1,:);
y=all_end_effector_p(2,:);
figure;plot(x,y,'o'); hold on;
plot(way_points(1,:),way_points(2,:),'ro'); hold on;

    %
x2=all_refer(1,:);
y2=all_refer(2,:);
figure;plot(x2,y2,'o'); hold on;
plot(way_points(1,:),way_points(2,:),'ro'); hold on;


figure(76)
plot(all_v_cartesian_target(3,:)); hold on;
plot(all_v_cartesian(3,:));
legend('理论速度')

