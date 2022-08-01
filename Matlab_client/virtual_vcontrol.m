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
% load all_end_effector_p.mat

close all;clear;clc;
warning('off')
%% Create the robot object

addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main')  


%% Caculate 
sum_e=0;
% theta_points=[ -0.4013,0.9539,0,-1.0490,0,1.1374,0].';
theta_points=[ -0.4013,0.9539,0,-1.0490,0,1.1374,0].';


way_points=[0.65;0.3;0.25];theta_points_final=[];
[useless, len_way_points]=size(way_points);
 theta_points=theta_points*180/pi;

%  [All_theta] = inverse_with_gesture(0.65,-0.3,0.25,init_theta1,init_theta2).'
%  All_theta=All_theta*pi/180
 
 
 
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
end

Ts=0.010;
T_tot=30;
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(theta_points,2)-1):T_tot;

[theta,theta_dot,theta_dotdot,pp] = cubicpolytraj(theta_points,tpts,tvec,...
                'VelocityBoundaryCondition', theta_d_des);

%% Start direct servo in joint space       
w=1.5; % motion constants, frequency rad/sec
A=pi/6; % motion constants, amplitude of motion
counter=0;all_acc=[]
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];v_filt=[];new_v_filt=[];
%% Initiate PIDDDDDDDDDDDDDDDDDDDDDDDDDD variables
k=[0.9089 0.8639 0.324];
k1=1;
k2=1;
k3=1;

%     k_cartesian = diag([100,100,100]*1*5);
%     b_cartesian = diag([100,100,100]*14*0.707*45/1000*5);   
%     H_inv = diag([1 1 1]/10/5);
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
%% Control loop   
all_v_cartesian_target=[];all_v_cartesian=[];alldt=[];all_contact_force_after=[];
all_target_joint_position_e=[];
All_v=[];EX_force_ori=[];all_F_contact=[];my_t=[{0} {0} {0} {0} {0} {0} {0}];all_f_attractor=[];all_end_effector_p=[];
robot_type=1;  all_jpos=[]; all_jtor=[];my_torque=[0 0 0 0 0 0 0];F_contact=0;all_joint_pos=[];all_target_end_effector_p=[];
all_dt_real=[];
q_t1dd=theta_dotdot(:,1);q_t1d=[0 0 0 0 0 0 0].';q_t1=[0 0 0 0 0 0 0].';intergal_goal=[];adjust=[];
refer_pos=[];
for q=1:size(theta_dot,2)
         [ pose, nsparam, rconf, jout ] = ForwardKinematics( theta(:,q).', robot_type );
        end_effector_p = pose(1:3,4);
        refer_pos=[refer_pos end_effector_p];
end
init_direction=refer_pos(:,1);init_direction_0=refer_pos(2,1);init_direction_end=refer_pos(2,end);
pause(3);accum_dt=0;all_xe=[]; all_xde=[];

init_pos=theta(:,1);feedback_joint_velocity_after=theta_dot(:,1);all_output=[];
dt_real=Ts;

rate_xdetjia=[];rate_target=[];
target_joint_position_next=[-0.401396453114820,0.954044131099899,7.47815770448098e-05,-1.04905153079427,4.89915666364060e-05,1.13728358894867,-6.17187693413365e-05];
% target_joint_position_next=[-0.432399711794801;0.933500000015965;0;-0.967999999993481;0;1.24009999998811;0].';

target_joint_velocity_next=[0 0 0 0 0 0 0].';all_this_point=[];last_p=target_joint_position_next;all_xetjia=[];v_filt=[0 0 0].';
tic;

for i=1:size(theta_dot,2)
%     target_joint_position=theta(:,1).';
%     target_joint_position_next=theta(:,1).';
%     target_joint_dotdot=theta_dotdot(:,1);
%     target_joint_velocity=theta_dot(:,1);
%     target_joint_velocity_next=theta_dot(:,1);
    pause(0.010)
    i
    if i == 1
        start = toc
    end
    accum_dt=accum_dt+dt_real;
    all_dt_real=[all_dt_real dt_real];
    dt=dt_real;
    
 
    [y,iy]=min(abs(refer_pos(2,:)-init_direction(2)));
%     ratio=(init_direction-init_direction_0)/(init_direction_end-init_direction_0);
    this_point=ceil(accum_dt/T_tot*size(theta_dot,2));
    
    if this_point >= size(theta_dot,2)
        this_point = size(theta_dot,2)-1;
    end   
    
    real_point=iy;
    if real_point >= size(theta_dot,2)
        real_point = size(theta_dot,2)-1;
    end
    if this_point >= size(theta_dot,2)
        this_point = size(theta_dot,2)-1;
    end
    all_this_point=[all_this_point this_point];

%     target_joint_position=theta(:,this_point).';
%     target_joint_position_next=theta(:,this_point+1).';
%   
%     target_joint_velocity=theta_dot(:,this_point);
%     target_joint_velocity_next=theta_dot(:,this_point+1);    
    
if i < 20
this_point=real_point;
end
    target_joint_position=target_joint_position_next;
    target_joint_position_next=theta(:,this_point+1).';
    target_joint_velocity=target_joint_velocity_next;
    target_joint_velocity_next=theta_dot(:,this_point+1);  
    
    this_p=init_pos;
    
    all_jpos=[all_jpos this_p];  

%% caculate outer force

    [ pose_c, nsparam_c, rconf_c, jout_c ] = ForwardKinematics( target_joint_position.', robot_type );
    target_end_effector_p = pose_c(1:3,4);
    target_end_effector_r = pose_c(1:3,1:3);
    all_target_end_effector_p=[all_target_end_effector_p target_end_effector_p];  %-----------
    
    %% ========Adj

% time update
        [Jac_this,A_mat_products] = Jacobian(target_joint_position,robot_type);
        J_dx_dq_this = Jac_this(1:3,:);
    [ pose_c2, nsparam_c, rconf_c, jout_c ] = ForwardKinematics( target_joint_position_next.', robot_type );
    target_end_effector_p_next = pose_c2(1:3,4);
    all_target_end_effector_p_next=[all_target_end_effector_p_next target_end_effector_p_next];



%         % controller
%         [Jac,A_mat_products] = Jacobian(init_pos,robot_type);
%         J_dx_dq = Jac(1:3,:);
        
        [Jac,A_mat_products] = Jacobian(target_joint_position.',robot_type);
        J_dx_dq = Jac(1:3,:);

        end_effector_p = init_direction;
        all_end_effector_p=[all_end_effector_p end_effector_p];
        
        end_effector_r = pose(1:3,1:3);

        f_bias=1*[0 0 0].';
        f_bias=end_effector_r*f_bias;
%         projection=end_effector_r*contact_force';
        contact_force_after = [0 0 0].';
 flagg=0;      
 thre=0;div=6;
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
    
 if i <10
     contact_force_after(3)=0;
 end
  
%  thre_2=4;
%   if contact_force_after(3)>thre_2
%       contact_force_after(3)=thre_2;
%   elseif contact_force_after(3)<-thre_2
%        contact_force_after(3)=-thre_2;
%   end
%   if contact_force_after(1)>thre_2
%       contact_force_after(1)=thre_2;
%   elseif contact_force_after(1)<-thre_2
%        contact_force_after(1)=-thre_2;
%   end 
%    if contact_force_after(2)>thre_2
%       contact_force_after(2)=thre_2;
%   elseif contact_force_after(2)<-thre_2
%        contact_force_after(2)=-thre_2;
%    end
  
   all_contact_force_after=[all_contact_force_after contact_force_after];
   



        v_cartesian_target=J_dx_dq*target_joint_velocity;
        all_v_cartesian_target=[all_v_cartesian_target v_cartesian_target];
   
        
        v_cartesian = J_dx_dq*feedback_joint_velocity_after;
        all_v_cartesian=[all_v_cartesian v_cartesian];

  
% v_filt=[v_filtered1 v_filtered2 v_filtered3].';
new_v_filt=[new_v_filt v_filt];
%% SVD
% [U,S,V] = svd(J_dx_dq)

        max=30;   % 8  8 5
        k=max/150; 
        if i <=1000 && i>850
            contact_force_after=[k*(i-850) 0 0].';
        elseif i >1000 && i<=1900
            contact_force_after=[max 0 0].';
        elseif i >1900 && i<=2050
            contact_force_after=[max-k*(i-1900) 0 0].';
%             contact_force_after=[0 0 0].';
        end
%         contact_force_after=[8 0 0].';
        adjust=[adjust contact_force_after];   
        
% contact_force_after=[0 0 0].';
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

%% 33333333333333333333333333333333333333333333333333333333333333333333333333333333

    a_d=(J_dx_dq*target_joint_velocity_next-J_dx_dq*target_joint_velocity)/dt;
    
%        xe=-target_end_effector_p+end_effector_p; %3 1
%        xde=-J_dx_dq*target_joint_velocity+v_filt;
       
       xe=-target_end_effector_p+end_effector_p+J_dx_dq*target_joint_velocity*dt; %3 1
       xde=-J_dx_dq*target_joint_velocity+v_filt+a_d*dt;       
       
       all_xe=[all_xe xe];
       all_xde=[all_xde xde];
       
      x_t1dd=H_inv*(contact_force_after-k_cartesian*xe-b_cartesian*xde);
      all_acc=[all_acc x_t1dd];
        equal_F=(-k_cartesian*xe-b_cartesian*xde);
        all_f_attractor=[all_f_attractor equal_F];
        
        
     xdetjia=-J_dx_dq*target_joint_velocity+v_filt+x_t1dd*dt;  %%% 
     v_filt=xdetjia+J_dx_dq*target_joint_velocity_next;
     rate_xdetjia=[rate_xdetjia xdetjia];
     rate_target=[rate_target J_dx_dq*target_joint_velocity_next];
     
     feedback_joint_velocity_after=pinv(J_dx_dq)*v_filt;
     feedback_joint_velocity_after(3)=0;feedback_joint_velocity_after(5)=0;feedback_joint_velocity_after(7)=0;
%      feedback_joint_velocity_after = satdq(dqlimit,feedback_joint_velocity_after);
    all_feedback_joint_velocity_after=[all_feedback_joint_velocity_after feedback_joint_velocity_after];
     

    this_theta_matrix=target_joint_velocity_next.';
    this_theta=num2cell(this_theta_matrix);

    this_zukang=num2cell(feedback_joint_velocity_after);

    
    t0=toc;
    dt_real=-start+t0;
    start=t0;
    
    
    fuck_total=J_dx_dq*feedback_joint_velocity_after*dt;
    init_direction=init_direction+fuck_total;
    init_pos=init_pos+feedback_joint_velocity_after*dt;
end

figure(52);

plot(adjust(1,:)); hold on;
plot(all_acc(1,:)*100); hold on; 
plot(new_v_filt(1,:)*300); hold on;
plot(all_end_effector_p(1,:)*100-65);
legend('filter force','acceleration','velocity','theortic position')


refer_pos=refer_pos.';
disp('Direct servo motion completed successfully')
warning('on')
figure(9);plot(refer_pos(:,1)); hold on; plot(all_end_effector_p(1,:));
legend('goal','real')
figure(10);plot(refer_pos(:,2)); hold on; plot(all_end_effector_p(2,:));
legend('goal','real')
figure(11);plot(refer_pos(:,3)); hold on; plot(all_end_effector_p(3,:));
legend('goal','real')
% figure(12);plot(all_dt_real)
figure(13);plot(rate_target(1,:)); hold on; plot(rate_xdetjia(1,:)); hold on; plot(new_v_filt(1,:))
figure(14);plot(rate_target(2,:)); hold on; plot(rate_xdetjia(2,:)); hold on; plot(new_v_filt(2,:))
figure(15);plot(rate_target(3,:)); hold on; plot(rate_xdetjia(3,:)); hold on; plot(new_v_filt(3,:))


