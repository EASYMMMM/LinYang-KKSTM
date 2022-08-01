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
    to_add(7)=0;
    theta_points=[theta_points to_add];
    
end
thetapi=theta_points*pi/180;
what=thetapi(:,1);
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
t=0;lastCmdTime=0;
OVER=0;

all_now_desired_theta=[];all_time=[];all_real_q=[];
all_now_desired_dtheta=[];all_this_point=[];
now_desired_dtheta_next=zeros(7,1);now_desired_theta_next=zeros(7,1);v_filt=zeros(3,1);
all_pos_table=[];

all_end_effector_p=[];all_delta=[];
    % trigger simulation正式开始循环
    Inter=robot.Intergate();
all_pos=[];all_F=[];this_F_att=[];
while OVER == 0
    THIS_ROUND=0;
    t=0;
    THETA_DOT=points_dot*pi/180;
    result_adjust=points*pi/180;
    q_now=result_adjust(:,1);
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
    %%
    pause(Ts);
    this_point=i
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
%%  解析解
% end_effector_p;
% pos_table = robot.position_of_table()';
% delta=pos_table-end_effector_p;
% rou_0=0.4;
% b=[0.475; -0.6; 0.2;]; %% assmue b is the center 
% zeta_att=1;miu_rep=1;
% kexi3=[];
% tic
% for xyz = 1:3
%     syms kexi
%     this_kexi1=[all_kexi3(xyz,:) kexi];
%     this_kexi1_T=[all_kexi3(xyz,:)'; kexi;];
%     [final_A]= find_A(i+1);
%     R=final_A'*final_A;
% % term_1=this_kexi1*R*(this_kexi1_T)/Ts^6;
%     term_1=this_kexi1*R*(this_kexi1_T);
% 
%     F_att=-zeta_att*(kexi+delta(xyz)-target_final3(xyz)); % 先不考虑距离过远的影像
%     rou_dist=(sum(pos_table.^2-b.^2))^0.5;
%     if rou_dist <= rou_0
%         F_rep=miu_rep*(1/rou_dist-1/rou_0)*1/(rou_dist)^2*1/(sum(pos_table.^2-b.^2)^0.5)*(kexi-b(xyz));
%     else
%         F_rep=0;
%     end
%     equation= term_1 == 0;
%     this_result_kxi=solve(equation,kexi);
%     this_result_kxi=vpa(this_result_kxi);
%     which=find(min(this_result_kxi-end_effector_p(xyz)));
%     use_kexi_1=this_result_kxi(which);
%     kexi3=[kexi3; use_kexi_1;];
%     kexi3=real(kexi3);
%     num_kexi3=double(kexi3);
% end
% all_kexi3=[all_kexi3 num_kexi3];
% toc
%% iteraction

end_effector_p;
all_end_effector_p=[all_end_effector_p end_effector_p];

pos_table = robot.position_of_table()';
all_pos_table=[all_pos_table pos_table];
delta=pos_table-end_effector_p;
all_delta=[all_delta delta];
rou_0=0.3;
b=[0.475; -0.6; 0.2;]; %% assmue b is the center 
zeta_att=1;miu_rep=1;
kexi3=[];
this_F=[];
for xyz = 1:3
    kexi = end_effector_p(xyz);
    this_kexi1=[all_kexi3(xyz,:)];
    this_kexi1_T=[all_kexi3(xyz,:)'];
    [final_A]= find_A_N3(i);
    R=final_A'*final_A;

    term_1=this_kexi1*R*(this_kexi1_T)/Ts^6;
%     F_att=-zeta_att*(kexi+delta(xyz)-target_final3(xyz)); % 先不考虑距离过远的影像
    F_att=-zeta_att*(kexi-target_final3(xyz)); % 先不考虑距离过远的影像
%     rou_dist=(sum((pos_table-b).^2))^0.5;
    [rou_dist, b] = find_distance(end_effector_p, 35, myspace);
    if rou_dist <= rou_0
        F_rep=miu_rep*(1/rou_dist-1/rou_0)*1/(rou_dist)^2*1/(sum(pos_table.^2-b.^2)^0.5)*(kexi-b(xyz));
    else
        F_rep=0;
    end
    
    equation=F_att+F_rep;
%     equation= term_1;
    this_F_att=[this_F_att; F_att;];
    kexi3=[kexi3; equation;];
    this_F=[this_F; F_rep;];
end
all_F=[all_F this_F];
kexi3_maxmin=kexi3./(sum(kexi3.^2))^0.5;
alpha=0.01;
F_kexi=alpha*kexi3_maxmin;
init_kexi=init_kexi+F_kexi;
all_kexi3=[all_kexi3 init_kexi];
for i0 = 1:2
init_theta=0;
xd=all_kexi3(1,end-2+i0);
yd=all_kexi3(2,end-2+i0);
zd=all_kexi3(3,end-2+i0);
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
    to_add(7)=0;
    
    
end
theta_points=[theta_points to_add];

delta_theta=(theta_points(:,end)-theta_points(:,end-1))/Ts*pi/180;

robot.ApplyControl(delta_theta, Ts); % velocity control
% if i == 10
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




% A=find_A_N3(10);
% R=A'*A;
% test_X=points3(1,1:10);
% test_X_T=test_X';
% test_X*R*test_X_T;



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
        
end



OVER=1;
end

%% 看结果
% sev=(all_now_desired_theta(:,2:end)-all_now_desired_theta(:,1:end-1))/Ts;
% for ww = 1:7
% figure(10+ww)
% plot(sev(ww,:)); hold on;
% plot(all_now_desired_dtheta(ww,:));
% end
% 
% desired_po=zeros(7,1);
% for w = 1:size(all_now_desired_dtheta,2)-1
%     desired_po=[desired_po sum(all_now_desired_dtheta(:,1:w+1),2)*Ts];
% end
%     
% for ww = 1:7
% figure(20+ww)
% adj=desired_po(ww,:)-(desired_po(ww,1)-all_now_desired_theta(ww,1));
% plot(adj); hold on;
% plot(all_now_desired_theta(ww,:)); hold on;
% plot(all_real_q(ww,:));
% legend('accumulate v','desired q','real q')
% end
% for see = 1:3
%     figure(see+30);
%     plot(all_end_effector_p(see,:)); hold on;
%     plot(all_pos_table(see,:));
%     legend('EEF','pos table')
% end

for see = 1:3
    figure(see+30);
    plot(all_end_effector_p(see,:)); hold on;
    plot(points3(see,:));hold on;
    plot(all_kexi3(see,:));hold on;
    plot(all_F(see,:));hold on;
    legend('EEF','points3','kexi','F rep')
end

% theta_points(:,1:2)=[];
% for see = 1:7
%     figure(see+20);
%     plot(all_real_q(see,:)); hold on;
%     plot(points(see,:));hold on;
%     
%     plot(theta_points(see,:));hold on;
%     
%     legend('EEF','points3','kexi')
% end

% for see = 1:3
%     figure(see+30);
%     plot(all_kexi3(see,:)); hold on;
%     plot(points3(see,:));
%     legend('kexi','mini jerk')
% end
% jb=[0.475; -0.6; 0.15;];
%     [distance, boundary] = find_distance(jb, 35, myspace)
    

     