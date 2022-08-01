%% Example of using KST class for interfacing with KUKA iiwa robots
% 看看z轴和斜切,记得看看，轻拿轻放，这个记录，我感觉Fz突然变大不太对。
% 0401我把力给改了，看看
% MMP, 居然要改4个地方， Jaccobin，ReferencePlane，FK，IK

close all;clear;clc;
warning('off')
TIMEE=[];

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
% theta_points=[0,0.772629915332932,0,-1.26703950381033,0,1.10192323444653,0];

% theta_points=[0,41.0257993937885,0,-78.6675247753817,0,54.8066758308298,0]*pi/180;%original saw with angle
% jj=[0;57.5174672529559;0;-57.8886953987786;0;-20.4061626517344;0]*pi/180;
% jj=[0;54.0304177140774;0;-68.9686947847894;0;-36.9991124988668;0]*pi/180;
jj=[0;48.2999158357778;0;-80.4847794064853;0;-42.7846952422630;0]*pi/180;
theta_points=jj.';


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
kalmanT = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalmanFX = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
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

% %30330329
%     k_cartesian = diag([100,100,100]*1*1)*1.3*5/2;
%     b_cartesian = diag([100,100,100]*14*0.707*45/1000*0.7*5*1.4/2*1.4);   
%     H_inv = diag([1 1 1]/10/5*3*2/2);
%30330329
%     k_cartesian = diag([100,100,100*4]*1*1)*1.3*5*2*1.5/2;
%     b_cartesian = diag([100,100,100*4*1.7]*14*0.707*45/1000*0.7*5*1.4/2*1.4);   
%     H_inv = diag([1 1 1*2/4/4]/10/5*3);

% %old high F attractor
    k_cartesian = diag([100,100,100*2]*1*1)*1.3*5*2*1.5/2
    b_cartesian = diag([100,100,100*2*1.7]*14*0.707*45/1000*0.7*5*1.4/2*1.4)
    H_inv = diag([1 1 1/4]/10/5*3)
%o20220402
%     k_cartesian = diag([100,100,100]*1*1)*1.3*5*2*1.5/2
%     b_cartesian = diag([100,100,100]*14*0.707*45/1000*0.7*5*1.4/2*1.4)
%     H_inv = diag([1 1 1]/10/5*3)
    w_n=(k_cartesian.*H_inv)^0.5;
    w_n=w_n(3,3);
    zeta=b_cartesian.*H_inv/2./w_n

    
    
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

count_loop=0;
rate_xdetjia=[];rate_target=[];
target_joint_position_next=theta_points;
init_pos=target_joint_position_next.';
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
    all_y_pro=[0];all_y=[];all_y_pro_fil=[0];
all_a=[]; all_b=[]; all_y=[];predict_test_y_KKK=[];FUCKact=[];


judge_EMG=0.13;  SONGSHOU=0;
%% CONTROAL LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP
all_theta=[]; all_theta_dot=[]; all_points=[]; all_points_dot=[]; posture=[];all_F_T_contact=[];all_aby=[];all_Tory=[];
output=[];predict_F=[];count_up=0;laststate=8;BIGTIME=2;ENDD=0;all_accum_dt=[];v_cartesian=zeros(3,1);BOTTOM=0;all_eula=[];
while FINISH == 0
    if ENDD == 1
        break
    end

        if round_sys == 25
            FINISH=1;
            break
        end

    STOP=0;
           round_sys=round_sys+1;
          contact_force_after=[0; 0; 0;];delete_point=1;accu_point=1;diminish=0;
          DOWN=0;UP=0;
          % up = 1代表累积了足够的countup，大UP代表现在正在上
          if count_up>15 && NEVER <=1 && laststate <=6
              up=1
              NEVER=NEVER+1
          else
              up=0
          end


          
          count_up=0;
            this_p_7=init_pos;
            [ pose, nsparam, rconf, jout ] = ForwardKinematics( this_p_7, robot_type );
            now_pos_3=pose(1:3,4)           
            [total_act way_points which_state_now myspace]= RL2m3m3(now_pos_3,angle_7,up,lock,last_space,laststate,round_sys);
            laststate=which_state_now
            lock
            last_space=myspace;
            if up == 1
                angle_comb=0;
            else     
                if which_state_now == 4 || which_state_now == 1 || which_state_now == 3 || which_state_now == 6
                    lock=1
                    angle_comb=angle_7;
                end
            end
            rotate_matrix_acc=[ [cosd(angle_comb), -sind(angle_comb) 0]; [sind(angle_comb), cosd(angle_comb) 0]; [0 0 1];];
            
            if total_act(1)-total_act(2) == 6
                DOWN=1
                lock = 0;
            end
            if total_act(1)-total_act(2) == -6
                UP=1
            end            
            
            way_points
            total_act 

                last_state=which_state_now;

            
            [useless, len_way_points]=size(way_points);
           theta_points= feedback_joint_position.';
            theta_points=theta_points*180/pi;

  %% use inverse kinematics       
  theta_d_des=[];
for z =1:len_way_points
    theta_d_des=[theta_d_des [0;0;0;0;0;0;0]];
end


BIGTIME=2;



T_tot=(size(way_points,2)-1)*BIGTIME;

all_BIGTIME=[all_BIGTIME BIGTIME];
all_accum_dt=[all_accum_dt accum_dt];
accum_dt=0;

if T_tot == 0
    disp('arrive')
    break
end

tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(way_points,2)-1):T_tot;
[ poseow, nsparam, rconf, jout ] = ForwardKinematics( feedback_joint_position, robot_type );
now_position3 = poseow(1:3,4);

processed_way_points=[now_position3 way_points(:,2)];

if DOWN == 1 || UP == 1 || up == 1
else
    processed_way_points(3,:) = [now_position3(3) now_position3(3) - 0.001/2];
end

if up == 1
    processed_way_points(3,:) = [now_position3(3) way_points(3,2)]
end

desired_v72=[zeros(7,2)];

[points,points_dot,points_dotdot,pp] = cubicpolytraj(processed_way_points,tpts,tvec,...
                'VelocityBoundaryCondition', zeros(3,2));   
% [points,points_dot,points_dotdot,pp] = cubicpolytraj(processed_way_points,tpts,tvec,...
%                 'VelocityBoundaryCondition', desired_v32);   
all_points=[all_points points];         
all_points_dot=[all_points_dot points_dot];

[hang,liehh]=size(points);    

for i_inv = 1 : liehh
    init_theta1=180;
    init_theta2=94;
%     init_theta2=5.5;
%     init_theta3=-angle_comb*180/pi;
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
    if round_sys <=100
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
         [ poseo, nsparam, rconf, jout ] = ForwardKinematics( theta_points(:,q).', robot_type );
        end_effector_po = poseo(1:3,4);
        refer_pos=[refer_pos end_effector_po];
end
all_refer=[all_refer refer_pos];
  
  
over=0;HELP=0;
for i_theta=1:1000
%%    control  
    i_theta;
    
%     if i_theta == 150
%         break
%     end
    
    if i_theta == 1
        start = toc
    end
    accum_dt=accum_dt+dt_real;
    if dt_real > 0.015
        dt_real=0.015;
    end
    
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

    
    if this_point >= length(tvec)/(size(way_points,2)-1)*3
        this_point = size(theta_dot,2);
        this_point
        break
    end
  [v_direction]= which_refer_v(total_act);
    
  %% stay at one place  

 if abs(contact_force_after(1))>5
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
    
    all_jpos=[all_jpos; this_p;];  
        [ pose_e, nsparame, rconfe, joute ] = ForwardKinematics( feedback_joint_position, robot_type );
        end_effector_p = pose_e(1:3,4);
        [a1 b1 y1]=inverse_angle(pose_e(1:3,1:3));
        eula=rotm2eul(pose_e(1:3,1:3));
        all_eula=[all_eula; eula;];
        all_aby=[all_aby; [a1 b1 y1];];
        angle_7=eula(1)*180/pi;
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
        F_T_contact=pinv(Jac.')*my_torque;
    all_F_T_contact=[all_F_T_contact F_T_contact]; 
    
F_filtered1 = kalmanx(F_contact(1));
F_filtered2 = kalmany(F_contact(2));
F_filtered3 = kalmanz(F_contact(3));
  
F_filt=[F_filtered1 F_filtered2 F_filtered3].';
new_f=[new_f F_filt];

        %% judge stay at one place       
        end_effector_r = pose_e(1:3,1:3);

        f_bias=[0 0 0].';
%         f_bias=[-1 0 -14].';
%         f_bias=end_effector_r*f_bias;
%         projection=end_effector_r*contact_force';
        contact_force_after = (F_filt-f_bias);
 flagg=0;      
 thre=4;div=1;
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
    
%  if i_theta <10
%      contact_force_after(3)=0;
%      contact_force_after(1)=0;
%      contact_force_after(2)=0;
%  end
  %%
 if end_effector_p(3) < 0.29 && up == 0
%       if end_effector_p(3) < 0.12 && up == 0
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
     
     target_end_effector_p(1)=target_joint_position_3_ori(1);
     target_joint_velocity=final_new_target_velocity_7;
 end
 
  if  round_sys == 12 
%     if round_sys == 4 || round_sys == 8
%         count_up=20;
%     end
     level=0;
     output=[output -1];
     STOP=STOP+1;
      if STOP >= 5
         output=[output 1];
      end    
 else
     output=[output 0];
 end
    

    
    if STOP>=1
        disp('STOPP')
        if SONGSHOU == 0
            mark_round=round_sys;
        end
        
        if SONGSHOU==1
            break
        end
        
        if output(end) ~= -1 && SONGSHOU == 0
            SONGSHOU=1
            STOP=0;
        end       
        
    end
    
    if count_up > 15 && up == 0 && NEVER <=1 && laststate <=6
        disp('i want to go up')
        break
    end

%%  
if up == 1 || DOWN == 1 || UP == 1
    disp('NO UP and Down')
    count_up =0;
end

if size(new_f,2)>5
    if sum(new_f(3,end-3:end))>0*4
        BOTTOM=1;
    end
end

% !!!!!!!!!!!直接锁死z轴, 力的修复
if which_state_now == 4 || which_state_now == 1 || which_state_now == 3 || which_state_now == 6
    new_F=rotate_matrix_acc.'*contact_force_after;
    new_F_pro=[new_F(1); 0; 0;];
    back_F=rotate_matrix_acc*new_F_pro;
    contact_force_after=back_F;
elseif which_state_now == 2 || which_state_now == 5
    contact_force_after(2) = 0;
end

if F_filt(end) > 20
    HELP =1;
end


if new_f(3,end) > 40 || UP == 1 || DOWN == 1
    contact_force_after(3)=0;
elseif BOTTOM == 1 && round_sys <= 40
    contact_force_after(3)=-0.1;
elseif BOTTOM == 1 && round_sys > 40
    contact_force_after(3)=-0.1;
elseif BOTTOM == 0
    contact_force_after(3)=-0.1;
end

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
     
     
     
%      feedback_joint_velocity_after=pinv(J_dx_dq)*v_filt;
  
     
%% y rot
        if end_effector_p(3) > 0.285
               k_ydof=100*1.3*2.5;
               d_ydof=100*14*0.707*45/1000*0.7*5*1.4/2*1.4;
               h_ydof=1/10/5*3*2;
               no_bais_T5=F_T_contact(6);
               T5_filtered = kalmanT(no_bais_T5);
               if T5_filtered < 1.5 && T5_filtered > -1.5
                   Tory = 0;
               elseif T5_filtered > 1.5
                   Tory = T5_filtered - 1.5;
               elseif T5_filtered < -1.5
                   Tory = T5_filtered + 1.5;
               end
               
               all_Tory=[all_Tory Tory];
               v_6=Jac*feedback_joint_velocity_after;
               a_ydof=h_ydof*(Tory*10-k_ydof*eula(1)-d_ydof*v_6(6));
               v_ydof=v_6(6)+a_ydof*dt;
               feedback_joint_velocity_after=pinv(Jac)*[v_filt; 0; 0; v_ydof;];
        else
            feedback_joint_velocity_after=pinv(Jac)*[v_filt; 0; 0; 0;];
        end 
        posture=[posture Jac*feedback_joint_velocity_after];
        
        
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
    if future_pos_3(3)<0.2
        disp('ERROR ! range is out of limit-2')
        ENDD=1;
        break
    end

    all_feedback_joint_velocity_after=[all_feedback_joint_velocity_after feedback_joint_velocity_after];

    this_theta_matrix=target_joint_velocity_next.';
    this_theta=num2cell(this_theta_matrix);

    this_zukang=num2cell(feedback_joint_velocity_after);

    feedback_joint_velocity_after=[0 0 0 0 0 0 0].';
    this_zukang=num2cell(feedback_joint_velocity_after);
    
        my_t=iiwa.sendJointsVelocitiesExTorques(this_zukang);



    
    
    init_pos=this_p.';
    init_direction=end_effector_p;
    
    fuck_total=J_dx_dq*feedback_joint_velocity_after*dt;
    init_direction_theory=init_direction_theory+fuck_total;
    all_init_direction_theory=[all_init_direction_theory init_direction_theory];
    
    t0=toc;
    dt_real=-start+t0;
    start=t0;    

end


end
figure;plot(all_aby(:,1)); hold on; 
plot(all_Tory); hold on;
plot(all_contact_force_after(end,:))
legend('a1','torque','F')
        
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
plot(way_points(1,:),way_points(2,:),'r'); 
refer_pos;
    %
x2=all_refer(1,:);
y2=all_refer(2,:);
figure;plot(x2,y2,'o'); hold on;
plot(way_points(1,:),way_points(2,:),'r');


figure(76)
plot(all_v_cartesian_target(3,:)); hold on;
plot(all_v_cartesian(3,:));
legend('理论速度','实际速度')


% FX
figure(45)
plot(all_target_end_effector_p_next(1,:)-0.6);hold on;
plot(all_end_effector_p(1,:)-0.6);hold on;
plot(new_f(1,:)/300); hold on
plot(all_contact_force_after(1,:)/100)

% FX
figure(45)
plot(all_target_end_effector_p_next(1,:)-0.6);hold on;
plot(all_end_effector_p(1,:)-0.6);hold on;
plot(new_f(1,:)/300); hold on 
plot(all_contact_force_after(1,:)/100)
% FX FZ
figure(46)
plot(all_end_effector_p(1,:)-0.45-0.2);hold on;
plot(all_target_end_effector_p_next(3,:)-0.2); hold on;
plot(all_end_effector_p(3,:)-0.2); hold on;
plot(new_f(1,:)/800/2); hold on
plot(new_f(3,:)/800/2); hold on
legend('real_x','target_x','real_z','real_Fx','real_Fz')

% FX FZ
figure(47)
plot(all_end_effector_p(1,:)-0.45-0.1);hold on;
plot(all_target_end_effector_p_next(3,:)-0.1); hold on;
plot(all_end_effector_p(3,:)-0.1); hold on;
plot(all_f_attractor(1,:)/800/2); hold on
plot(all_f_attractor(3,:)/800/2); hold on
plot(all_contact_force_after(1,:)/800/2); hold on
plot(all_contact_force_after(3,:)/20); hold on
plot(new_v_filt(3,:))
legend('real_x','target_x','real_z','F_attractorx','F_attractorz','control_FX','control_FZ','VZ')


all_end_effector_p(3,6320)-all_end_effector_p(3,7342)
all_end_effector_p(3,4044)-all_end_effector_p(3,5237)



figure;
plot(all_end_effector_p(1,:),all_end_effector_p(2,:))
