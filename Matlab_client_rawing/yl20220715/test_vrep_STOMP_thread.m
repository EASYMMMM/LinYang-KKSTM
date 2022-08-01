clear;
clc;close all;
% this is the main loop file for runing in the vrep, and we also have
% another file in GPR with STOMP. author : Lin YANG ，但是一些变量的正确性还需要检查
%% Parameters
% Controller gain
Kp_joint = eye(7)*20;
Kp_cart = eye(3)*15;
% Redundancy gain
k0 = 10; %0 for standard (non collision-free) motion planning 
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

t_client_EMG_local=tcpip('localhost',30000,'NetworkRole','client','TimeOut',200);%与本地主机建立连接，端口号为30000，作为客户机连接。
t_client_EMG_local.InputBuffersize=51200;
t_client_EMG_local.OutputBufferSize=51200;
disp(['未打开！',datestr(now)])
fopen(t_client_EMG_local);%与一个服务器建立连接，直到建立完成返回，否则报错。
disp(['已打开！',datestr(now)])



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
% T_tot=(size(way_points,2)-1)*1;
% tvec = 0:Ts:T_tot;
% tpts = 0:T_tot/(size(way_points,2)-1):T_tot;
% 
% % % 续上速度
% desired_v32=[zeros(3,size(way_points,2))];
% desired_v72=[zeros(7,size(way_points,2))];
% [points,points_dot,points_dotdot,pp] = cubicpolytraj(way_points,tpts,tvec,...
%                 'VelocityBoundaryCondition', desired_v32);   
            
T_tot=3;
t=0;lastCmdTime=0;
OVER=0;


    q = robot.GetState();
    now_position=directKinematics(q);
    fwrite(t_client_EMG_local,[q' q'],'double');
all_now_desired_theta=[];all_time=[];all_real_q=[];
all_now_desired_dtheta=[];all_this_point=[];
now_desired_dtheta_next=zeros(7,1);now_desired_theta_next=zeros(7,1);v_filt=zeros(3,1);
while  t_client_EMG_local.BytesAvailable == 0
    
    pause(0.01)
    if t_client_EMG_local.BytesAvailable > 0
        data_recv_all = fread(t_client_EMG_local,t_client_EMG_local.BytesAvailable/8,'double');%    disp(size(data_recv));
        bre=find(data_recv_all == 666);
        data_recv_theta=data_recv_all(bre+1:end);
        data_table=data_recv_all(1:bre-1);
 
        result_adjust=reshape(data_recv_theta,7,length(data_recv_theta)/7);
        THETA_DOT=[(result_adjust(:,2:end)-result_adjust(:,1:end-1))/Ts zeros(7,1)];
        break
    end
end

    % trigger simulation正式开始循环
    Inter=robot.Intergate();
all_pos=[];
while OVER == 0
    THIS_ROUND=0;
    t=0;
%     refer_pos=[];
% for q=1:size(theta_points,2)
%          [ poseo, nsparam, rconf, jout ] = ForwardKinematics( theta_points(:,q).', 1 );
%         end_effector_po = poseo(1:3,4);
%         refer_pos=[refer_pos end_effector_po];
% end



for i=1:300
%     if i_theta == 1
%         start = toc
%     end
%     accum_dt=accum_dt+dt_real;
%     all_dt_real=[all_dt_real dt_real];
%     dt=dt_real;
% 
%     count_loop=count_loop+1;
% 
%         this_point=ceil(accum_dt/T_tot*size(theta_dot,2));
%         direction=which_refer_direction(total_act);
%         [~,iy]=min(abs(refer_pos(direction,:)-init_direction(direction)));
%         
%          real_point=iy;
% 
%          
%         if real_point >= size(theta_dot,2)
%             real_point = size(theta_dot,2);
%         end 




    
    if  t_client_EMG_local.BytesAvailable>0
        disp('I receive the new refer trajectory!')
        data_recv_all = fread(t_client_EMG_local,t_client_EMG_local.BytesAvailable/8,'double');%    disp(size(data_recv));
        if data_recv_all(end) == 87654321
            OVER=1;
            break
        end
        bre=find(data_recv_all == 666);
        data_recv_theta=data_recv_all(bre+1:end);
        data_table=data_recv_all(1:bre-1);
        
        result=reshape(data_recv_theta,7,length(data_recv_theta)/7);
        
        over_run=this_point-ceil(size(THETA_DOT,2)*0.75)+25;
        result_left=result(:,over_run+1:end);
        processed_way_points=[now_desired_theta result(:,over_run)];
        
%         bound_v7=[now_desired_dtheta zeros(7,1)];
        bound_v7=[now_desired_dtheta (result(:,over_run+1)-result(:,over_run))/Ts];
        tvec = 0:Ts:over_run*Ts;
        tpts=0:over_run*Ts:over_run*Ts;
%         tpts = 0:over_run/T_tot;
        [points,points_dot,points_dotdot,pp] = cubicpolytraj(processed_way_points,tpts,tvec,...
                'VelocityBoundaryCondition', bound_v7);   
%            figure; plot(points(1,:))
            
%         delete_zero=find(sum(points_dot,1)==0);
%         delete_zero=delete_zero(1);
%         if delete_zero>0
%             points2=points(:,2:delete_zero-1);
%             points_dot2=points_dot(:,1:delete_zero-1);
%         else
%             points2=points;
%             points_dot2=points_dot;
%         end
        
        result_adjust=[points(:,2:end) result_left];
        THETA_DOT=[points_dot(:,2:end) (result(:,over_run+2:end)-result(:,over_run+1:end-1))/Ts];

        
%         THETA_DOT=[(result(:,2:end)-result(:,1:end-1))/Ts zeros(7,1)];
%         OVER=1;
        break
    end

    rq = robot.GetState();
    all_real_q=[all_real_q rq];
    %%
    pause(Ts);
    this_point=i;
%     this_point=ceil(t/T_tot*size(THETA_DOT,2));
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
    
%     jj=[];init=all_now_desired_theta(:,1);
%     for k = 1:length(all_now_desired_dtheta)
%         this=all_now_desired_dtheta(:,k);
%         init=init+this*Ts;
%         jj=[jj init];
%     end


        [Jac,A_mat_products] = Jacobian(feedback_joint_position,1);
        J_dx_dq = Jac(1:3,:);
        J_dpose_dq = Jac(4:6,:);
        desired_v_7=now_desired_dtheta;
        desired_v_6=Jac*now_desired_dtheta;

%% attance control
%     contact_force_after=[0; 0; 0;];
    contact_force_after=robot.GetF();
    
    contact_force_after=contact_force_after';
    if length(contact_force_after)~=3
        contact_force_after=zeros(3,1);
    end
    contact_force_after=zeros(3,1);
    
    target_joint_velocity_next=now_desired_dtheta_next;

    target_joint_velocity=now_desired_dtheta;
    
    [ pose1, nsparam, rconf, jout ] = ForwardKinematics( now_desired_theta, 1 );
    target_end_effector_p=pose1(1:3,4);
    [ pose2, nsparam, rconf, jout ] = ForwardKinematics( rq, 1 );
    end_effector_p=pose2(1:3,4);

    if i == 1
        dt = Ts;
    end
    a_d=(J_dx_dq*target_joint_velocity_next-J_dx_dq*target_joint_velocity)/dt;
       xe=-target_end_effector_p+end_effector_p+J_dx_dq*target_joint_velocity*dt;
       xde=-J_dx_dq*target_joint_velocity+v_filt+a_d*dt;
      x_t1dd=H_inv*(contact_force_after-k_cartesian*xe-b_cartesian*xde);
      
        equal_F=(-k_cartesian*xe-b_cartesian*xde);
     xdetjia=-J_dx_dq*target_joint_velocity+v_filt+x_t1dd*dt;  

     v_filt=xdetjia+J_dx_dq*target_joint_velocity_next;
%      feedback_joint_velocity_after=pinv(Jac)*[v_filt; J_dpose_dq*now_desired_dtheta;];
     feedback_joint_velocity_after=pinv(Jac)*[desired_v_6(1:end);];

%     robot.ApplyControl(feedback_joint_velocity_after, Ts); % velocity control
    robot.ApplyControl(now_desired_dtheta, Ts); % velocity control
%     robot.ApplyPosi(now_desired_theta);

%% PID addF
q = robot.GetState();
        [ poseow, nsparam, rconf, jout ] = ForwardKinematics( q, 1 );
        where_robot_3 = poseow(1:3,4);    
        
m_table=16.67*0.7; b_table=152.776*2; k_table=975*3;
    w_n=(k_table/m_table)^0.5;
    zeta=b_table/m_table/2/w_n

pos = robot.position_of_table();
all_pos=[all_pos pos'];
velocity = robot.velocity_of_table();
a_e=-velocity(3)/dt;
v_e=-velocity(3);
position_z=where_robot_3(3);
data_table
PEIHE=1;
if i/size(THETA_DOT,2) <= 1/(length(data_table)+1)
    x_tar = give_tableF(data_table(1),position_z);
elseif i/size(THETA_DOT,2) <= 2/(length(data_table)+1)
    x_tar = give_tableF(data_table(2),position_z);
elseif i/size(THETA_DOT,2) <= 3/(length(data_table)+1)
    x_tar = give_tableF(data_table(3),position_z);
else
    x_tar = give_tableF(data_table(3),position_z);
end
x_e=-(pos(3)-x_tar);
F_e=m_table*a_e+b_table*v_e+k_table*x_e;
F_sensor=robot.GetF();
robot.Applyhand2(0,0,F_e);

    
    if this_point>ceil(size(THETA_DOT,2)*0.75) && THIS_ROUND == 0% it is time to send current position
        THIS_ROUND=1;
        
        now_position=directKinematics(q);
        fwrite(t_client_EMG_local,[now_desired_theta' q'],'double');
    end
        
        
        lastCmdTime = currentCmdTime;
        t = t+dt;
        
end
end

%% 看结果
% sev=(all_now_desired_theta(:,2:end)-all_now_desired_theta(:,1:end-1))/Ts;
% for ww = 1:7
% figure(10+ww)
% plot(sev(ww,:)); hold on;
% plot(all_now_desired_dtheta(ww,:));
% end
% 
desired_po=zeros(7,1);
for w = 1:size(all_now_desired_dtheta,2)-1
    desired_po=[desired_po sum(all_now_desired_dtheta(:,1:w+1),2)*Ts];
end
    
for ww = 1:7
figure(20+ww)
adj=desired_po(ww,:)-(desired_po(ww,1)-all_now_desired_theta(ww,1));
plot(adj); hold on;
plot(all_now_desired_theta(ww,:)); hold on;
plot(all_real_q(ww,:));
legend('accumulate v','desired q','real q')
end



     