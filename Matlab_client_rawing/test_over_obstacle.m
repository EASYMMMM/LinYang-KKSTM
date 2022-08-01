%% Example of using KST class for interfacing with KUKA iiwa robots
% 测试一下仿真转移到KST，只过一个障碍

close all;clear;clc;
warning('off')


% t_server_self=tcpip('0.0.0.0',30000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
% t_server_self.InputBuffersize=100000;
% disp(['未打开！',datestr(now)])
% fopen(t_server_self);%打开服务器，直到建立一个TCP连接才返回；
% disp(['已打开！',datestr(now)])

%%
% IP_remote_IMU = "192.168.11.1"; 
% port_remote_IMU = 5000;
% IP_local_IMU = "192.168.11.2"; 
% port_local_IMU = 5000;
% Role_IMU = 'client';
% t_server_IMU = tcpip(IP_remote_IMU,port_remote_IMU,...
%                 'NetworkRole',Role_IMU,...
%                 'LocalPort',port_local_IMU,...
%                 'TimeOut',20,...
%                 'InputBufferSize',8192);
% 
% t_server_IMU.InputBuffersize=100000;
% 
% disp(['未打开！',datestr(now)])
% fopen(t_server_IMU);%打开服务器，直到建立一个TCP连接才返回；
% disp(['已打开！',datestr(now)])

data_all_IMU=[];count_right_IMU=0;

%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main-raw')  
raw_points=200;FLAG=0;predict_y=[];
wrong=0;how_many_points_0=[];kinds=12;how_many_points_1=[];
how_many_points_2=[];all_middle=[];
how_many_points_3=[];how_many_points_4=[];how_many_points_5=[];how_many_points_11=[];
how_many_points_6=[];how_many_points_7=[];how_many_points_8=[];how_many_points_9=[];how_many_points_10=[];
    recv_once_0=[];recv_once_1=[];recv_once_2=[];recv_once_3=[];recv_once_4=[];recv_once_5=[];recv_once_6=[];
     recv_once_6=[];recv_once_7=[];recv_once_8=[];recv_once_9=[];recv_once_10=[];recv_once_11=[];recv_once_12=[];current_9xyz=[];
all_9xyz=[];final_EMGxyz=[];all_x_intension=[];all_y_intension=[];all_z_intension=[];all_current_7_avg=[];

flag = 0;count_which_matrix=0;

count2=0;count_x=0;

total_rece_0=[];total_rece_1=[];total_rece_2=[];total_rece_6=[];ALL_LEN_OF_RECV2=[];
total_rece = [];total_head=0;total_rece_3=[];total_rece_4=[];total_rece_5=[];all_delta_theta=[];
all_delta_wan_y=[];all_delta_wan_x=[];all_delta_wan_z=[];
all_delta_jian_y=[];all_delta_jian_x=[];all_delta_jian_z=[];
all_delta_zhou_y=[];all_delta_zhou_x=[];all_delta_zhou_z=[];
all_sum_wan_y=[];all_sum_wan_x=[];all_sum_wan_z=[];



disp('Start a connection with the server')
%% Start a connection with the server
flag=iiwa.net_establishConnection();
if flag==0
  return;
end
pause(1);
disp('Moving first joint of the robot using a sinusoidal function')
    
%% high level initialize 
CHANGE=0;
now_pos_3=[0.1 -0.45 0.2]';
[lastq,lastR,total_act way_points which_state_now myspace cartis_obs OBSTACLE]= RL2m3m3_maze_ok(now_pos_3,0,CHANGE,0,0,0,0,[],0);
last_state=which_state_now;
last_space=myspace;
last_q=lastq;
last_R=lastR;


%% Go to initial position - 测试一个障碍
relVel=0.15; % the relative velocity
 way_points=[[0.35; -0.6; 0.15] [0.475; -0.6; 0.5;]];
theta_points=[];
for i0 = 1:2
init_theta=0;
xd=way_points(1,i0);
yd=way_points(2,i0);
zd=way_points(3,i0);
    init_theta1=180;
    init_theta2=0;
    [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
    count_no=0;
                while isempty(All_theta) && count_no<120
                init_theta2=init_theta2+1;
                count_no=count_no+1;
                 [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2)';
                end  
    [hang,lie]=size(All_theta);
    if i0 == 1
        temp=All_theta(:,end);
    else
        temp=theta_points(:,end);
    end
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
    to_add(7)=30;
    theta_points=[theta_points to_add];
    
end
thetapi=theta_points;
what=thetapi(:,1);
what(end)=30;
what=what*pi/180;
theta_points2=num2cell(what)
iiwa.movePTPJointSpace(theta_points2, relVel); % move to initial configuration

all_gamma=[];all_beta=[];all_alpha=[];

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

%% Start direct servo in joint space       
Ts = 0.01;
T_tot=(size(way_points,2)-1)*3;
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(way_points,2)-1):T_tot;

% % 续上速度
desired_v32=[zeros(3,size(way_points,2))];
[points3,points_dot3,points_dotdot3,pp3] = cubicpolytraj(way_points,tpts,tvec,...
                'VelocityBoundaryCondition', desired_v32);   
            

all_q=[];
t=0;lastCmdTime=0;
OVER=0;

all_now_desired_theta=[];all_time=[];all_real_q=[];
all_now_desired_dtheta=[];all_this_point=[];
now_desired_dtheta_next=zeros(7,1);now_desired_theta_next=zeros(7,1);v_filt=zeros(3,1);
all_pos_table=[];all_q_init=[];

all_end_effector_p=[];all_delta=[];

all_pos=[];all_F=[];this_F_att=[];v_control=zeros(6,1);
target_joint_velocity=zeros(7,1);
end_effector_p=now_pos_3;
FUTURE=3;
high_loop=0;
q_init=what;
all_v_control=[];all_control_signal=[];all_qd_dot=[];all_points_dot3=[];
dists_1=[];dists_2=[];


%% Start direct servo in joint space       
iiwa.realTime_startVelControlJoints();all_output=[];
counter=0;
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];v_filt=[];new_v_filt=[];
%% Initiate PIDDDDDDDDDDDDDDDDDDDDDDDDDD variables
    k_cartesian = diag([100,100,100]*1*1)*1.3*5;
    b_cartesian = diag([100,100,100]*14*0.707*45/1000*0.7*10/2*2.5);   
    H_inv = diag([1 1 1]/10/5*2);
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
%get(t_server);
data_all = [];count_right = 0;
    count_self = 0;
%代表那个大循环   
    go=0;
%%  please prepare for starting

    
%% Control loop   
all_v_cartesian_target=[];all_v_cartesian=[];alldt=[];all_contact_force_after=[];
all_target_joint_position_e=[];
All_v=[];EX_force_ori=[];all_F_contact=[];my_t=[{0} {0} {0} {0} {0} {0} {0}];all_f_attractor=[];all_end_effector_p=[];
robot_type=1;  all_jpos=[]; all_jtor=[];my_torque=[0 0 0 0 0 0 0];F_contact=0;all_joint_pos=[];all_target_end_effector_p=[];
all_dt_real=[];


dt_real=Ts;

rate_xdetjia=[];rate_target=[];
 current_combine=[];predict_x=0;predict_y=0;predict_z=0;out_all_predict_z=[];out_all_predict_x=[];out_all_predict_y=[];
 all_new_EMG2=[];all_new_EMG=[];all_predict_z=[];all_predict_x=[];all_predict_y=[];
 count_self=0; sychronize=[];  Final=[];out_predict_x=0;out_predict_y=0;out_predict_z=0;



data_all_IMU=[];count_right_IMU=0;
all_toc=[];

q_init=what;
accum_dt=0;


tic
for i=1:size(points_dot3,2)*2
    i
    
%             if  t_server_self.BytesAvailable>0
%                     data_recv_self = fread(t_server_self,t_server_self.BytesAvailable/8,'double');%    disp(size(data_recv));
%                     count_self = count_self + 1;
%                     which_head_self=find(88887<=data_recv_self);
%                     which_head2_self=which_head_self(end);
%                     this_frame_self=zeros(9,1);
%                     
%                     this_IMU=data_recv_self(which_head2_self:end);
%                     this_frame_self(1:length(this_IMU))=this_IMU;
%                     data_all(:,count_self) = this_frame_self;
%             end
%     if  t_server_IMU.BytesAvailable>0
%         data_recv_IMU = fread(t_server_IMU,t_server_IMU.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
%         
%         which_head_IMU=find(88887<=data_recv_IMU);
%         if ~isempty(which_head_IMU)
%             which_head2_IMU=which_head_IMU(end);
%             this_frame_IMU=data_recv_IMU(which_head2_IMU:end);
%             if length(this_frame_IMU) == 76
%                 
%                 count_right_IMU = count_right_IMU + 1;
%                 data_all_IMU(:,count_right_IMU) = this_frame_IMU;
%                 this_frame_IMU_1=this_frame_IMU(2:26); %竖着
%                 this_frame_IMU_2=this_frame_IMU(27:51);%竖着
%                 this_frame_IMU_3=this_frame_IMU(52:end); %竖着
%             end
%         end
%     end



 %% CONTROL   
    if i == 1
        start = toc
    end
    accum_dt=accum_dt+dt_real;
    all_dt_real=[all_dt_real dt_real];
    dt=dt_real;
    
    this_p=iiwa.getJointsPos();
    this_p=cell2mat(this_p);
    
    all_jpos=[all_jpos; this_p;];  
    
vector=points3(:,end)-points3(:,1);      
unit_vector=vector/norm(vector);
now_vec=end_effector_p-points3(:,1);   
angle=acos(dot(vector,now_vec)/(norm(vector)*norm(now_vec)));
now_vec_proj=now_vec*cos(angle);
run_rate=norm(now_vec_proj)/norm(vector);
i_theta=ceil(run_rate*size(points_dot3,2))
  
  
    
    
    if i_theta >= size(points_dot3,2)
        i_theta = size(points_dot3,2);
    end
    
%% caculate outer force
    
    %% ========Adj

   

%         % controller
        [Jac,A_mat_products] = Jacobian(this_p,robot_type);
        J_dx_dq = Jac(1:3,:);
        
my_torque=cell2mat(my_t).';
new_torque=[new_torque; cell2mat(my_t);];

    F_contact=1*pinv(J_dx_dq.')*my_torque;
    all_F_contact=[all_F_contact F_contact];       %%!!! ''

    
F_filtered1 = kalmanx(F_contact(1));
F_filtered2 = kalmany(F_contact(2));
F_filtered3 = kalmanz(F_contact(3));
  
F_filt=[F_filtered1 F_filtered2 F_filtered3].';
new_f=[new_f F_filt];


         [ pose, nsparam, rconf, jout ] = ForwardKinematics( this_p, robot_type );
        end_effector_p = pose(1:3,4);





%% real time iteraction

all_end_effector_p=[all_end_effector_p end_effector_p];

pos_table = end_effector_p;
all_pos_table=[all_pos_table pos_table];
delta=pos_table-end_effector_p;
all_delta=[all_delta delta];

    q = this_p';
    all_q=[all_q q];
    J67=Jac;
    J=J67(1:3,:);
    
    
    J_pinv = pinv(J);
    Kp_joint = eye(7)*2;
    k0 = 0;    
    q0_dot=zeros(7,1);
    for double_obs = 1:2
        if double_obs == 1
            this_obs=35;
            [dists,grads] = distancesAndGrads_tableU(q, this_obs, delta, myspace);
            dists_1=[dists_1 dists];
            [~,index] = min(dists);
            q_each_dot = k0*grads(index,:)';
        else
            this_obs=79;
            [dists,grads] = distancesAndGrads_tableU(q, this_obs, delta, myspace);
            dists_2=[dists_2 dists];
            [~,index] = min(dists);
            q_each_dot = k0*grads(index,:)';
        end
        
        q0_dot=q0_dot+q_each_dot;
    end
    F=J*q0_dot;
    all_F=[all_F F];
    qd_dot = J_pinv * points_dot3(:,i_theta) + q0_dot;
%     qd_dot = J_pinv * points_dot3(:,i) + (eye(7)-J_pinv*J)*q0_dot;


    q_init = q_init + qd_dot*Ts;          %Euler integration
    all_q_init=[all_q_init q_init];
    q_err = q_init - q;
    control_signal = Kp_joint*q_err;
    q_control_dot = control_signal + qd_dot;
    all_qd_dot=[all_qd_dot qd_dot];
    all_control_signal=[all_control_signal control_signal];
    v_control=J67*q_control_dot;
    all_v_control=[all_v_control v_control];

    
%     fuck=J_pinv * points_dot3(:,i_theta);
    
    
     %% SAFE
    future_pos_7=q_init+q_control_dot*dt;
    for each_joint = 1:7
        if (future_pos_7(each_joint) <= qmin(each_joint)) || (future_pos_7(each_joint) >= qmax(each_joint))
            disp('ERROR ! range is out of limit')
            v_control(each_joint)=0;
        end
    end
    future_pos_3=end_effector_p+J_dx_dq*q_control_dot*dt;
    if future_pos_3(3)<0.05
        disp('ERROR ! range is out of limit')
        break
    end


    feedback_joint_velocity_after=q_control_dot;
%     feedback_joint_velocity_after=[0 0 0 0 0 0 0].';
    this_zukang=num2cell(feedback_joint_velocity_after);

        my_t=iiwa.sendJointsVelocitiesExTorques(this_zukang);

    
    t0=toc;
    dt_real=-start+t0;
    start=t0;
    all_toc=[all_toc t0];


end

%% turn off server
% iiwa.realTime_stopImpedanceJoints()
iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer()


figure;
plot(all_F');hold on;

all_hf=[];
for alf= 1:size(all_F,2)
    helif=norm(all_F(:,alf));
    all_hf=[all_hf helif];
end
plot(all_hf)
legend('x','y','z','all')

% 
figure;
plot(dists_1')
legend('1','2','3','4','5','6');

figure;
plot(dists_2')
legend('1','2','3','4','5','6');
% pause(10)
% fwrite(t_server_IMU,[88888.888,7654321],'double');%写入数字数据，每次发送360个double
% fwrite(t_server_self,[88888.888,7654321],'double');%写入数字数据，每次发送360个double
% fclose(t_server_self);
% fclose(t_server_IMU);
% 
% sychronize;
% save('20220501yl3_stop.mat','sychronize')
% save('20220501yl3_stop_all.mat')

