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
[lastq,lastR,total_act way_points which_state_now myspace cartis_obs OBSTACLE]= RL2m3m3_maze(now_pos_3,0,CHANGE,0,0,0,0,[],0);
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
%                     if now_y<0
%                         init_theta2=init_theta2+1;
%                         init_theta1=180-count_no;
%                     elseif now_y>0
%                         init_theta2=init_theta2+1;
%                         init_theta1=180+count_no;
%                     else
%                         init_theta2=init_theta2+1;
%                         init_theta1=180+count_no;  
%                     end
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
thetapi=theta_points*pi/180;
what=thetapi(:,1);
what(end)=30*pi/180;
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
T_tot=(size(way_points,2)-1)*1;
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(way_points,2)-1):T_tot;

% % 续上速度
desired_v32=[zeros(3,size(way_points,2))];



desired_v72=[zeros(7,size(way_points,2))];
[points3,points_dot3,points_dotdot3,pp3] = cubicpolytraj(way_points,tpts,tvec,...
                'VelocityBoundaryCondition', desired_v32);   
            
[points,points_dot,points_dotdot,pp] = cubicpolytraj(theta_points,tpts,tvec,...
                'VelocityBoundaryCondition', desired_v72);   
            
T_tot=3;


all_q=[];

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
q_init=what;
all_v_control=[];all_control_signal=[];all_qd_dot=[];all_points_dot3=[];
dists_1=[];dists_2=[];

while OVER == 0
    
    
    THIS_ROUND=0;
    t=0;
    THETA_DOT=points_dot*pi/180;
    result_adjust=points*pi/180;
    q_init=result_adjust(:,1);
    q_final=result_adjust(:,end);
    [ pose1, nsparam, rconf, jout ] = ForwardKinematics( result_adjust(:,end), 1 );
    target_final3=pose1(1:3,4);
        init_kexi=points3(:,1);
    all_kexi3=[ points3(:,1)]; % all_kexi
%     init_kexi=points3(:,7);
%     all_kexi3=[ points3(:,1:7)]; % all_kexi
for i=1:size(points_dot,2)

    
    
    rq = robot.GetState();
    all_real_q=[all_real_q rq];
    this_point=i;
    all_this_point=[all_this_point this_point];
    if this_point >=size(THETA_DOT,2)
        this_point=size(THETA_DOT,2)-1;
    end
            currentCmdTime = robot.GetLastCmdTime();
            all_time=[all_time currentCmdTime];
        dt = (currentCmdTime-lastCmdTime)/1000;
        % get states feedback
    feedback_joint_position=robot.GetState();
    feedback_joint_velocity=robot.GetV();
    
    
    if i ==1
        now_desired_theta=result_adjust(:,1);
        now_desired_dtheta=THETA_DOT(:,1);
    else
        now_desired_dtheta=now_desired_dtheta_next;
        now_desired_theta=now_desired_theta_next;
    end
    
    now_desired_dtheta_next=THETA_DOT(:,this_point+1);
    now_desired_theta_next=result_adjust(:,this_point+1);
    all_now_desired_theta=[all_now_desired_theta now_desired_theta];

    all_now_desired_dtheta=[all_now_desired_dtheta now_desired_dtheta];
    
        [Jac,A_mat_products] = Jacobian(feedback_joint_position,1);
        J_dx_dq = Jac(1:3,:);
        J_dpose_dq = Jac(4:6,:);
        desired_v_7=now_desired_dtheta;
        desired_v_6=Jac*now_desired_dtheta;

    [ pose1, nsparam, rconf, jout ] = ForwardKinematics( now_desired_theta, 1 );
    target_end_effector_p=pose1(1:3,4);
    [ pose2, nsparam, rconf, jout ] = ForwardKinematics( rq, 1 );
    end_effector_p=pose2(1:3,4);

    if i == 1
        dt = Ts;
    end

%% iteraction
tic
end_effector_p;
all_end_effector_p=[all_end_effector_p end_effector_p];

pos_table = robot.position_of_table()';
all_pos_table=[all_pos_table pos_table];
delta=pos_table-end_effector_p;
all_delta=[all_delta delta];
rou_0=0.3;

zeta_att=1;miu_rep=1;
kexi3=[];
this_F=[];

    q = robot.GetState();
    all_q=[all_q q];
    [J67, A_mat_products] = Jacobian_table(q,1);
    J=J67(1:3,:);

    J_pinv = pinv(J);
    Kp_joint = eye(7)*20;
    k0 = 0.0001;    
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
%     miu=5;
%     if i<size(points_dot,2)
%         desired_i=i;
%         
%     else
%         desired_i=size(points_dot,2);
% %         qd_dot = miu * (q_final-q);
%     end
    qd_dot = J_pinv * points_dot3(:,i) + q0_dot;
 
%     qd_dot = J_pinv * points_dot3(:,i) + (eye(7)-J_pinv*J)*q0_dot;
    R=find_A(i); weight_R=0.001;
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

% q_control_dot = qd_dot;


robot.ApplyControl(q_control_dot, Ts); % velocity control
% if i == 3
%     break
% end
%% attance control
%     contact_force_after=[0; 0; 0;];
    contact_force_after=robot.GetF();
    
    contact_force_after=contact_force_after';
%     contact_force_after=contact_force_after;
    if length(contact_force_after)~=3
        contact_force_after=zeros(3,1);
    end
    contact_force_after=zeros(3,1);
    
    target_joint_velocity_next=now_desired_dtheta_next;

    target_joint_velocity=now_desired_dtheta;
    

    a_d=(J_dx_dq*target_joint_velocity_next-J_dx_dq*target_joint_velocity)/dt;
       xe=-target_end_effector_p+end_effector_p+J_dx_dq*target_joint_velocity*dt;
       xde=-J_dx_dq*target_joint_velocity+v_filt+a_d*dt;
      x_t1dd=H_inv*(contact_force_after-k_cartesian*xe-b_cartesian*xde);
      
        equal_F=(-k_cartesian*xe-b_cartesian*xde);
     xdetjia=-J_dx_dq*target_joint_velocity+v_filt+x_t1dd*dt;  

     v_filt=xdetjia+J_dx_dq*target_joint_velocity_next;
%      feedback_joint_velocity_after=pinv(Jac)*[v_filt; J_dpose_dq*now_desired_dtheta;];
     feedback_joint_velocity_after=pinv(Jac)*[desired_v_6(1:end);];
%     robot.ApplyControl(now_desired_dtheta, Ts); % velocity control
%     robot.ApplyControl(feedback_joint_velocity_after, Ts); % velocity control



%% PID addF
q = robot.GetState();
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

% PEIHE=1;
% if i/size(THETA_DOT,2) <= 1/(length(data_table)+1)
%     x_tar = give_tableF(data_table(1),position_z);
% elseif i/size(THETA_DOT,2) <= 2/(length(data_table)+1)
%     x_tar = give_tableF(data_table(2),position_z);
% elseif i/size(THETA_DOT,2) <= 3/(length(data_table)+1)
%     x_tar = give_tableF(data_table(3),position_z);
% else
%     x_tar = give_tableF(data_table(3),position_z);
% end
% x_e=-(pos(3)-x_tar);
% F_e=m_table*a_e+b_table*v_e+k_table*x_e;
% F_sensor=robot.GetF();
robot.Applyhand2(0,0,0);


        lastCmdTime = currentCmdTime;
        t = t+dt;
        toc
end



OVER=1;
end

%% 看结果
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
% all_q_init;
% for see = 1:7
%     figure(see+30);
%     plot(all_q_init(see,:)); hold on;
%     plot(all_q(see,:));
%     legend('desired q ','real q')
% end
%     
    

     