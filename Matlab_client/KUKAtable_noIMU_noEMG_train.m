%% Example of using KST class for interfacing with KUKA iiwa robots
%只有kuka，1，建议一旦不更新地图，就

close all;clear;clc;
warning('off')
TIMEE=[];
addpath('C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client_rawing\EMG_IMU')
addpath 'C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main'
data_all_IMU=[];count_right_IMU=0;

%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object

%% Start a connection with the server
flag=iiwa.net_establishConnection();
if flag==0
  return;
end
pause(1);

%% Go to initial position
relVel=0.15; % the relative velocity
theta_points=[-0.463647609000806,0.820126730911849,0,-1.17997019621945,0,1.14149572645849,0];
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
kalmanT = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
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
w=1.5; % motion constants, frequency rad/sec
A=pi/6; % motion constants, amplitude of motion
counter=0;
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];v_filt=[];new_v_filt=[];all_this_y_v=[];come_in=[];
%% Initiate PIDDDDDDDDDDDDDDDDDDDDDDDDDD variables
    k_cartesian = diag([100,100,100]*1*1)*1.3*5/2/2;
    b_cartesian = diag([100,100,100]*14*0.707*45/1000*0.7*5*1.4/2*1.4/2);   
    H_inv = diag([1 1 1]/10/5*3*2*2);

all_target_joint_position_3_ori=[];lock=0;
global_z=0;
   
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
accum_dt=0;all_xe=[]; all_xde=[];

feedback_joint_velocity_after=zeros(7,1);all_output=[];last_space=[{0} {0} {0} {0} {0} {0} {0}];
init_pos=theta_points.';
count_loop=0;
rate_xdetjia=[];rate_target=[];
target_joint_position_next=theta_points;
feedback_joint_position=target_joint_position_next;
target_joint_position=target_joint_position_next;all_this_point_after=[];
FINISH=0;round_sys=0; compared_p=[];compared_v=[];all_target_joint_position=[];all_real_joint_position=[];
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
    all_y_pro=[0];all_y_pro_fil=[0];
all_a31=[]; all_b31=[]; all_y31=[];predict_test_y_KKK=[];FUCKact=[];
all_y21=[];all_a21=[];all_b21=[];


judge_EMG=0.13;  SONGSHOU=0;
%% CONTROAL LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP
disp(' TIME TO GOOOOO')    
all_theta=[]; all_theta_dot=[]; all_points=[]; all_points_dot=[]; posture=[]; all_F_T_contact=[];all_aby=[];all_Tory=[];
output=[];predict_F=[];count_up=0;laststate=8;BIGTIME=2;ENDD=0;all_accum_dt=[];v_cartesian=zeros(3,1);
%0307
this_frame_self=zeros(9,1); all_EMG=[]; a31=0; b31=0; y31=0;
all_a23=[]; all_b23=[]; all_y23=[]; a23=0; b23=0; y23=0;
BOTTOM=0;last_v_filt=[0;0;0;];
CHANGE=0;final_v3=zeros(3,1);all_target_joint_velocity=[];all_a_d=[];
v_cartesian_target=zeros(3,1); target_joint_velocity=zeros(7,1);v_filt=[0;0;0;];
while FINISH == 0
    if ENDD == 1
        break
    end
go_into1=toc;    
%     if round_sys == 3
%             FINISH=1
%             break
%     end
    STOP=0;
    
           round_sys=round_sys+1;
          contact_force_after=[0; 0; 0;];delete_point=1;accu_point=1;diminish=0;
          DOWN=0;UP=0;
          
          count_up=0;
            this_p_7=init_pos;
            [ pose, nsparam, rconf, jout ] = ForwardKinematics_table( theta_points, robot_type );
%             [ pose, nsparam, rconf, jout ] = ForwardKinematics_table( this_p_7, robot_type );
            now_pos_3=pose(1:3,4)    
            
            CHANGE=0;
            now_pos_3=[0.1 -0.45 0.2]';
            [lastq,lastR,total_act way_points which_state_now myspace]= RL2m3m3_maze(now_pos_3,0,CHANGE,0,0,0,0);
            all0=[];
            for ii = 1:size(lastq,2)
                if all(lastq(:,ii)==0)
                    all0=[all0 ii];
                end
            end
% 根据相机扫描决定要不要CHANGE环境           
            
            if CHANGE == 0
                [lastq,lastR,total_act way_points which_state_now myspace]= RL2m3m3_maze(now_pos_3,0,CHANGE,0,0,0,0);
                CHANGE=1
            else
                [lastq,lastR,total_act way_points which_state_now myspace]= RL2m3m3_maze(now_pos_3,myspace,CHANGE,lastq,lastR);
            end
            if total_act == 11
                FINISH=1
                break
            end
            laststate=which_state_now;
            last_space=myspace;
            way_points
            total_act 

                last_state=which_state_now;

            
            [useless, len_way_points]=size(way_points);
           theta_points= feedback_joint_position.';
            theta_points=theta_points*180/pi;

  %% use inverse kinematics       

BIGTIME=3;

T_tot=BIGTIME
% T_tot=(size(way_points,2)-1)*BIGTIME
all_BIGTIME=[all_BIGTIME BIGTIME];
all_accum_dt=[all_accum_dt accum_dt];
accum_dt=0;

if T_tot == 0
    disp('arrive')
    break
end

tvec = 0:Ts:T_tot;
% tpts = 0:T_tot/(size(way_points,2)-1):T_tot;
tpts = 0:T_tot/(2-1):T_tot;
[ poseow, nsparam, rconf, jout ] = ForwardKinematics_table( feedback_joint_position, robot_type );
now_position3 = poseow(1:3,4);

processed_way_points=[now_position3 way_points(:,2)];
% % 续上速度
% desired_v32=[final_v3 zeros(3,1)]
% desired_v72=[feedback_joint_velocity_after zeros(7,1)]
% 续上速度
desired_v32=[v_cartesian_target zeros(3,1)]
desired_v72=[target_joint_velocity zeros(7,1)]
% desired_v72=[zeros(7,2)];
[points,points_dot,points_dotdot,pp] = cubicpolytraj(processed_way_points,tpts,tvec,...
                'VelocityBoundaryCondition', desired_v32);   
            
% [points,points_dot,points_dotdot,pp] = cubicpolytraj(processed_way_points,tpts,tvec,...
%                 'VelocityBoundaryCondition', zeros(3,2));   
all_points=[all_points points];         
all_points_dot=[all_points_dot points_dot];

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
    which_need=abs(All_theta(7,:)) >= 140;
    need_plus=All_theta(7,which_need);
    if need_plus < -140
        All_theta(7,which_need)=need_plus+180;
    elseif need_plus > 140
        All_theta(7,which_need)=need_plus-180;
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
    if round_sys <= 20
        to_add(7)=0;
%     elseif round_sys >= 20 && lock == 1
%         to_add(7)=angle_comb*180/pi;
    end
    theta_points=[theta_points to_add];
end
theta_points=theta_points.*pi/180;
theta_points(:,1)=[];
theta=theta_points;
theta_dot=[desired_v72(:,1) (theta(:,3:end)-theta(:,1:end-2))/Ts/2 desired_v72(:,2)];
all_theta=[all_theta theta];
all_theta_dot=[all_theta_dot theta_dot];



refer_pos=[];
for q=1:size(theta_points,2)
         [ poseo, nsparam, rconf, jout ] = ForwardKinematics_table( theta_points(:,q).', robot_type );
        end_effector_po = poseo(1:3,4);
        refer_pos=[refer_pos end_effector_po];
end
all_refer=[all_refer refer_pos];
%test
if length(total_act) > 2 
    break_theta=theta;
    break_theta_dot=theta_dot;
    break_refer_pos=refer_pos;
    break_points=points;
    break_points_dot=points_dot;
end




over=0;pre_y=0; KEEP = 0;temp_intent=[];temp_EMG_left=[];WAIT=0;HELP=0;

go_into2=toc;
delta_big_time=go_into2-go_into1

for i_theta=1:1000
%%    control  
    i_theta;
    if i_theta == 1
        start = toc;
    end
    accum_dt=accum_dt+dt_real;
    all_dt_real=[all_dt_real dt_real];
    dt=dt_real;

    count_loop=count_loop+1;

        this_point=ceil(accum_dt/T_tot*size(theta_dot,2));
        total_act12=total_act(1:2);
        direction=which_refer_direction(total_act12); % 这个函数和你的构型有关系
        [~,iy]=min(abs(refer_pos(direction,:)-init_direction(direction)));
        
         real_point=iy;

         
        if real_point >= size(theta_dot,2)
            real_point = size(theta_dot,2);
        end 

    
    if this_point >= length(tvec)*1.5
        this_point = size(theta_dot,2);
        disp('TIMEOUT')
        break
    end
%   [v_direction]= which_refer_v(total_act12); % 这是干嘛的
    
  %% stay at one place  
 if abs(contact_force_after(direction))>20
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
            FUCKact(end)=[];
            come_in(end)=[];
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
    
    if i_theta == 1
        target_joint_position=theta(:,1).';
        target_joint_velocity=theta_dot(:,1);
    end
    
    all_target_joint_velocity=[all_target_joint_velocity target_joint_velocity];
    all_target_joint_position=[all_target_joint_position; target_joint_position;];
%% caculate outer force

    [ pose_c, nsparam_c, rconf_c, jout_c ] = ForwardKinematics_table( target_joint_position.', robot_type );
    target_end_effector_p = pose_c(1:3,4);
    all_target_end_effector_p=[all_target_end_effector_p target_end_effector_p];  %-----------
    
    %% ========Adj

% time update
        [Jac_this,A_mat_products] = Jacobian(target_joint_position,robot_type);
        J_dx_dq_this = Jac_this(1:3,:);
    [ pose_c2, nsparam_c, rconf_c, jout_c ] = ForwardKinematics_table( target_joint_position_next.', robot_type );
    target_end_effector_p_next = pose_c2(1:3,4);
    all_target_end_effector_p_next=[all_target_end_effector_p_next target_end_effector_p_next];
    
    this_p=iiwa.getJointsPos();
    this_p=cell2mat(this_p);
    feedback_joint_position=this_p;
    angle_7=feedback_joint_position(end);
    all_jpos=[all_jpos; this_p;];  
        [ pose_e, nsparame, rconfe, joute ] = ForwardKinematics_table( feedback_joint_position, robot_type );
        end_effector_p = pose_e(1:3,4);
        all_end_effector_p=[all_end_effector_p end_effector_p];
        [a1 b1 y1]=inverse_angle(pose_e(1:3,1:3));
        all_aby=[all_aby; [a1 b1 y1];];
        
        all_feedback_joint_position=[all_feedback_joint_position; feedback_joint_position;];
        contact_force = F_contact;
%         % controller
        [Jac,A_mat_products] = Jacobian(feedback_joint_position,robot_type);
        J_dx_dq = Jac(1:3,:);
        
my_torque=cell2mat(my_t).';
new_torque=[new_torque; cell2mat(my_t);];

    F_contact=1*pinv(J_dx_dq.')*my_torque;
    F_T_contact=pinv(Jac.')*my_torque;
    F_contact=1*pinv(J_dx_dq.')*my_torque;
    all_F_contact=[all_F_contact F_contact];       %%!!! 
        F_T_contact=pinv(Jac.')*my_torque;
    all_F_T_contact=[all_F_T_contact F_T_contact]; 
    
F_filtered1 = kalmanx(F_contact(1));
F_filtered2 = kalmany(F_contact(2));
F_filtered3 = kalmanz(F_contact(3));
  
F_filt=[F_filtered1 F_filtered2 F_filtered3].';
new_f=[new_f F_filt];
        %% judge stay at one place        
        end_effector_r = pose_e(1:3,1:3);
        f_bias=[0 0 18].';
        f_bias=end_effector_r*f_bias;
        contact_force_after = (F_filt-f_bias);
        if i_theta<10
            contact_force_after(3)=0;
        end
        
 flagg=0;      
 thre=1;div=2;
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
    


%%  
contact_force_after=zeros(3,1);

   all_contact_force_after=[all_contact_force_after contact_force_after];
        v_cartesian_target=J_dx_dq*target_joint_velocity;
        all_v_cartesian_target=[all_v_cartesian_target v_cartesian_target];
        v_cartesian = J_dx_dq*feedback_joint_velocity_after;
        all_v_cartesian=[all_v_cartesian v_cartesian];



%% 33333333333333333333333333333333333333333333333333333333333333333333333333333333

    a_d=(J_dx_dq*target_joint_velocity_next-J_dx_dq*target_joint_velocity)/dt;
    all_a_d=[all_a_d a_d];
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
     new_v_filt=[new_v_filt v_filt];

     last_v_filt=v_filt;
     rate_xdetjia=[rate_xdetjia xdetjia];
     rate_target=[rate_target J_dx_dq*target_joint_velocity_next];
     
     feedback_joint_velocity_after=pinv(Jac)*[v_filt; 0; 0; 0;];
       feedback_joint_velocity_after = satdq(dqlimit,feedback_joint_velocity_after);
     %% SAFE
    future_pos_7=this_p.'+target_joint_velocity_next*dt;
    for each_joint = 1:7
        if (future_pos_7(each_joint) <= qmin(each_joint)) || (future_pos_7(each_joint) >= qmax(each_joint))
            future_pos_7
            disp('ERROR ! range is out of limit-1')
            ENDD=1;
            feedback_joint_velocity_after(each_joint)=0;
            break
        end
    end
    future_pos_3=end_effector_p+J_dx_dq*target_joint_velocity_next*dt;
    if future_pos_3(3)<0.02
        disp('ERROR ! range is out of limit-2')
        ENDD=1;
        break
    end

    all_feedback_joint_velocity_after=[all_feedback_joint_velocity_after feedback_joint_velocity_after];

    this_theta_matrix=target_joint_velocity_next.';
    this_theta=num2cell(this_theta_matrix);
    final_v3=Jac*feedback_joint_velocity_after;
    final_v3=final_v3(1:3);
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
    
     % 测试用打断
     if this_point_after > 200 && length(total_act) > 2
         break
     end
         
         
end  
end


iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer()
figure(66)
plot(all_target_joint_velocity(2,:)); hold on;
plot(all_feedback_joint_velocity_after(2,:)); hold on;
figure(67)
plot(all_feedback_joint_position(:,2)); hold on;
plot(all_target_joint_position(:,2))
figure(69)
plot(new_v_filt(2,:)); hold on;
plot(all_v_cartesian_target(2,:))
figure(68)
jb=J_dx_dq*(all_target_joint_velocity(:,2:end)-all_target_joint_velocity(:,1:end-1));
plot(jb(2,:))
