clear;
clc;close all;
% this is test file for 2 order control
%% Parameters

% Integration Time Step
Ts = 0.01;

% Controller type
controller = "joint";
    k_cartesian = diag([100,100,100*2]*1*1)*1.3*5*2*1.5/2
    b_cartesian = diag([100,100,100*2*1.7]*14*0.707*45/1000*0.7*5*1.4/2*1.4)
    H_inv = diag([1 1 1/4]/10/5*3)  
    KEY_CONTROL=0; %是否键盘那控制
%% 初始化
robot = VrepConnector(19999,0.01);
addpath('D:\summer research\TRO\Matlab_client_rawing\EMG_IMU')
addpath 'D:\summer research\TRO\Motion-Planning-for-KUKA-LBR-main'
addpath 'D:\summer research\TRO\Matlab_client'
addpath 'D:\summer research\TRO\STOMP_done\Intro-to-Robo-Proj-master'



CHANGE=0;
now_pos_3=[0.1 -0.45 0.2]';
[lastq,lastR,total_act way_points which_state_now myspace cartis_obs OBSTACLE]= RL2m3m3_maze(now_pos_3,0,CHANGE,0,0,0,0,[]);
last_state=which_state_now;
last_space=myspace;
last_q=lastq;
last_R=lastR;

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
    [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
 theta_points=All_theta(:,2); 

All_theta=All_theta*pi/180;
what=All_theta(:,2);
what(end)=20*pi/180;
robot.ApplyPosi2(what);


q = robot.GetState();
while(norm(directKinematics(q) - way_points(:,1)) > 0.1 )
    now_2=directKinematics(q);
    q = robot.GetState();
end
now_3=directKinematics(q);
qd = robot.GetState();

    %%  initialization 2
    noo=robot.Inital();
t=0;lastCmdTime=0;
OVER=0;

all_now_desired_theta=[];all_time=[];all_real_q=[];
all_now_desired_dtheta=[];all_this_point=[];
now_desired_dtheta_next=zeros(7,1);now_desired_theta_next=zeros(7,1);v_filt=zeros(3,1);
all_pos_table=[];all_q_init=[];

all_end_effector_p=[];all_delta=[];
    % trigger simulation正式开始循环
    Inter=robot.Intergate();
all_pos=[];all_F=[];this_F_att=[];v_control=zeros(6,1);
target_joint_velocity=zeros(7,1);
end_effector_p=now_pos_3;
FUTURE=3;
high_loop=0;
q_init=what;dists_1=[];dists_2=[];
all_v_control=[];all_control_signal=[];all_qd_dot=[];all_points_dot3=[];points_dot3=zeros(3,1);
while OVER == 0
    
    CHANGE=1;
    start_position=end_effector_p;

    [lastq,lastR,total_act way_points which_state_now myspace cartis_obs OBSTACLE]= RL2m3m3_maze(start_position,last_space,CHANGE,last_q,last_R,0,0,OBSTACLE, cartis_obs);

    high_loop=high_loop+1;
    all_space=cell2mat(myspace(:,3));
    position_of_obs=all_space(cartis_obs,:);
     
    if size(way_points,2) > FUTURE
        start_end_points=[start_position way_points(:,FUTURE)];
    else
        start_end_points=[start_position way_points(:,end)];
        OVER=1;
    end
    data_table=[]; % 1 table up 0 middle -1 down
    for each_s = 1:FUTURE
        each_state=total_act(each_s);
        if each_state <=132
            data_table=[data_table 0];
        elseif each_state <=198
            data_table=[data_table -1];
        else
            data_table=[data_table 1];
        end
    end
            
    data_table        
    total_act
    start_end_points
    
    [~,which_state1]=min(sum(abs(position_of_obs-start_end_points(:,1).'),2));
     position_of_obs(which_state1,:)=[1000 1000 1000];
    [~,which_state2]=min(sum(abs(position_of_obs-start_end_points(:,2).'),2));
    obs1=cartis_obs(which_state1);
    obs2=cartis_obs(which_state2);
    v_end01=(start_end_points(:,end)-start_end_points(:,1))/norm(start_end_points(:,end)-start_end_points(:,1));
    
    v_end=0.1*v_end01;
%     desired_v32=[ v_control(1:3) v_end];
    desired_v32=[ points_dot3(:,end) v_end];
%     desired_v32=[zeros(3,size(start_end_points,2))];
    T_tot=(size(start_end_points,2)-1)*0.6;
    tvec = 0:Ts:T_tot;
    tpts = 0:T_tot/(size(start_end_points,2)-1):T_tot;

 [points3,points_dot3,points_dotdot3,pp3] = cubicpolytraj(start_end_points,tpts,tvec,...
                'VelocityBoundaryCondition', desired_v32);     
    t=0;

    all_q=[];

for i=1:size(points3,2)
    %%
    currentCmdTime = robot.GetLastCmdTime();
    all_time=[all_time currentCmdTime];
    dt = (currentCmdTime-lastCmdTime)/1000;
    if i == 1
        dt = Ts;
    end

%% iteraction

pos_table = robot.position_of_table()';
all_pos_table=[all_pos_table pos_table];

q = robot.GetState();
 all_q=[all_q q];
[ pose2, nsparam, rconf, jout ] = ForwardKinematics( q, 1 );
end_effector_p=pose2(1:3,4);
all_end_effector_p=[all_end_effector_p end_effector_p];
delta=pos_table-end_effector_p;
all_delta=[all_delta delta];

   
    [J67, A_mat_products] = Jacobian_table(q,1);
    J=J67(1:3,:);
% Kinematic control of joint motion
%     qd_dot = inverseDifferentialKinematicsAug(qd, xd, obs, k0);
    
    J_pinv = pinv(J);
    Kp_joint = eye(7)*20;
    k0 = 0.001;   
    q0_dot=zeros(7,1);
  for double_obs = 1:2
        if double_obs == 1
            this_obs=79;
            [dists,grads] = distancesAndGrads_tableU(q, this_obs, delta, myspace);
            dists_1=[dists_1 dists];
            [~,index] = min(dists);
            q_each_dot = k0*grads(index,:)';
        else
            this_obs=35;
            [dists,grads] = distancesAndGrads_tableU(q, this_obs, delta, myspace);
            dists_2=[dists_2 dists];
            [~,index] = min(dists);
            q_each_dot = k0*grads(index,:)';
        end
        
        q0_dot=q0_dot+q_each_dot;
    end
    F=J67*q0_dot;
    all_F=[all_F F];
    qd_dot = J_pinv * points_dot3(:,i) + q0_dot;
%     qd_dot = pinv(J67) * [points_dot3(:,i); zeros(3,1);] + q0_dot;
    all_points_dot3=[all_points_dot3 points_dot3(:,i)];
    
%     qd_dot = J_pinv * points_dot3(:,i) + (eye(7)-J_pinv*J)*q0_dot;
    R=find_A(i); weight_R=0;
    temp_minJ=[];
    for kexii = 1:7
        temp_minJ=[temp_minJ; all_q(kexii,:)*R*(all_q(kexii,:)');];
    end
    qd_dot=qd_dot+weight_R*temp_minJ;
%     jointSpaceController
    q_init = q_init + qd_dot*Ts;          %Euler integration
    all_q_init=[all_q_init q_init];
    q_err = q_init - q;
    control_signal = Kp_joint*q_err;
    q_control_dot = control_signal + qd_dot;
    all_qd_dot=[all_qd_dot qd_dot];
    all_control_signal=[all_control_signal control_signal];
    v_control=J67*q_control_dot;
    all_v_control=[all_v_control v_control];

    
%     jb=pinv(J67) * [points_dot3(:,i); zeros(3,1);];
      
robot.ApplyControl(q_control_dot, Ts); % velocity control  
% robot.ApplyControl(jb, Ts); % velocity control
% if i == 1 && high_loop == 2
%     OVER = 1;
%     break
% end
        Jac=J67;
        J_dx_dq = J;

%% attance control
%     contact_force_after=[0; 0; 0;];
    contact_force_after=robot.GetF();
    
    contact_force_after=contact_force_after';
%     contact_force_after=contact_force_after;
    if length(contact_force_after)~=3
        contact_force_after=zeros(3,1);
    end
    contact_force_after=zeros(3,1);
    
    target_joint_velocity_next=q_control_dot;
    [ pose1, nsparam, rconf, jout ] = ForwardKinematics( q_init, 1 );
    target_end_effector_p=pose1(1:3,4);

    a_d=(J_dx_dq*target_joint_velocity_next-J_dx_dq*target_joint_velocity)/dt;
       xe=-target_end_effector_p+end_effector_p+J_dx_dq*target_joint_velocity*dt;
       xde=-J_dx_dq*target_joint_velocity+v_filt+a_d*dt;
      x_t1dd=H_inv*(contact_force_after-k_cartesian*xe-b_cartesian*xde);
      
        equal_F=(-k_cartesian*xe-b_cartesian*xde);
     xdetjia=-J_dx_dq*target_joint_velocity+v_filt+x_t1dd*dt;  

     v_filt=xdetjia+J_dx_dq*target_joint_velocity_next;
%      feedback_joint_velocity_after=pinv(Jac)*[v_filt; J_dpose_dq*now_desired_dtheta;];
     feedback_joint_velocity_after=pinv(Jac)*[v_filt; 0; 0; 0;];
%     robot.ApplyControl(now_desired_dtheta, Ts); % velocity control
%     robot.ApplyControl(feedback_joint_velocity_after, Ts); % velocity control

target_joint_velocity=target_joint_velocity_next;

%% PID addF

        [ poseow, nsparam, rconf, jout ] = ForwardKinematics( q, 1 );
        where_robot_3 = poseow(1:3,4);    
        
m_table=16.67*0.7; b_table=152.776*2; k_table=975*3;
    w_n=(k_table/m_table)^0.5;
    zeta=b_table/m_table/2/w_n;


all_pos=[all_pos pos_table'];
velocity = robot.velocity_of_table();
a_e=-velocity(3)/dt;
v_e=-velocity(3);
position_z=where_robot_3(3);


if i/size(points3,2) <= 1/(length(data_table))
    x_tar = give_tableF(data_table(1),position_z);
elseif i/size(points3,2) <= 2/(length(data_table))
    x_tar = give_tableF(data_table(2),position_z);
elseif i/size(points3,2) <= 3/(length(data_table))
    x_tar = give_tableF(data_table(3),position_z);
else
    x_tar = give_tableF(data_table(3),position_z);
end
x_e=-(pos_table(3)-x_tar);
F_e=m_table*a_e+b_table*v_e+k_table*x_e;
F_sensor=robot.GetF();
robot.Applyhand2(0,0,0);


        lastCmdTime = currentCmdTime;
        t = t+dt;

end
end

%% 看结果
all_F=all_F(1:3,:);
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
% figure;
% plot(all_v_control(1:3,:)');

% all_posi_init=[];
% for eachq =1:size(all_q_init,2)
%     [ pose2, nsparam, rconf, jout ] = ForwardKinematics( all_q_init(:,eachq), 1 );
%     enF=pose2(1:3,4);
%     all_posi_init=[all_posi_init enF];
% end

% 
% all_q_init;
% for see = 1:7
%     figure(see+30);
%     plot(all_q_init(see,:)); hold on;
%     plot(all_q(see,:));
%     legend('desired q ','real q')
% end
%     
%     all_F
% all_v_control
%      all_end_effector_p


% for see = 1:3
%     figure(see+30);
%     plot(all_F(see,:)); hold on;
%     plot(all_v_control(see,:)); hold on;
%     plot(all_end_effector_p(see,:));
%     legend('desired q ','real q')
% end