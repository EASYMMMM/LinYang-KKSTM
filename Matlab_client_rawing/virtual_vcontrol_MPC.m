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

addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main-raw')  
sum_e=0;

%% Start direct servo in joint space       
w=1.5; % motion constants, frequency rad/sec
A=pi/6; % motion constants, amplitude of motion
counter=0;all_acc=[]
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];v_filt=[];new_v_filt=[];
%% Initiate PIDDDDDDDDDDDDDDDDDDDDDDDDDD variables
k=[0.9089 0.8639 0.324];

    k_cartesian = diag([100,100,100]*1*1)*1.3*5/4;
    b_cartesian = diag([100,100,100]*14*0.707*45/1000*0.7*5*1.4/4*1.4);   
    H_inv = diag([1 1 1]/10/5*3*4);

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
intergal_goal=[];adjust=[];

init_direction=[0.65,0,0.2].';
accum_dt=0;all_xe=[]; all_xde=[];

feedback_joint_velocity_after=zeros(7,1);all_output=[];
init_pos=[0,0.772629915332932,0,-1.26703950381033,0,1.10192323444653,0].';

count_loop=0;
rate_xdetjia=[];rate_target=[];
target_joint_position_next=[0,0.772629915332932,0,-1.26703950381033,0,1.10192323444653,0];


FINISH=0;round=0; compared_p=[];compared_v=[];all_target_joint_position=[];all_real_joint_position=[];

target_joint_position=target_joint_position_next;
target_joint_velocity_next=[0 0 0 0 0 0 0].';all_this_point=[];last_p=target_joint_position_next;all_xetjia=[];v_filt=[0 0 0].';
tic;
Ts=0.020;
dt_real=Ts;
while FINISH == 0
    round=round+1;
          
            this_p_7=init_pos;
            [ pose, nsparam, rconf, jout ] = ForwardKinematics( this_p_7, robot_type );
         
            
            [total_act way_points which_state_now myspace]= RL2m3m3(pose(1:3,4));
            total_act
            which_state_now
            [useless, len_way_points]=size(way_points);
             way_points
            theta_points=target_joint_position.';
           
            theta_points=theta_points*180/pi;
 
  [hang,lie]=size(way_points);
for i_inv = 2 : lie
    init_theta1=180;
    init_theta2=0;
    xd=way_points(1,i_inv);
    yd=way_points(2,i_inv);
    zd=way_points(3,i_inv);
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
theta_points=theta_points.*pi/180



theta_d_des=[];
for z =1:len_way_points
    theta_d_des=[theta_d_des [0;0;0;0;0;0;0]];
end


T_tot=(size(theta_points,2)-1)*4;
if T_tot == 0
    disp('arrive')
    break
end
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(theta_points,2)-1):T_tot;

[theta,theta_dot,theta_dotdot,pp] = cubicpolytraj(theta_points,tpts,tvec,...
                'VelocityBoundaryCondition', theta_d_des);

  if round == 1
      refer_theta=theta;
  end
      

            
clear_loop=0;   accum_dt=0; target_joint_position_next=theta(:,1).';

for i=1:300
%     for i=1:size(theta_dot,2)
%     target_joint_position=theta(:,1).';
%     target_joint_position_next=theta(:,1).';
%     target_joint_dotdot=theta_dotdot(:,1);
%     target_joint_velocity=theta_dot(:,1);
%     target_joint_velocity_next=theta_dot(:,1);
    pause(0.010);

    i
    if i == 1
        start = toc
    end
    accum_dt=accum_dt+dt_real;
    all_dt_real=[all_dt_real dt_real];
    dt=dt_real;
    count_loop=count_loop+1;
    clear_loop=clear_loop+1;

    
%     [y,iy]=min(abs(refer_pos(2,:)-init_direction(2)));
%     ratio=(init_direction-init_direction_0)/(init_direction_end-init_direction_0);
    this_point=ceil(accum_dt/T_tot*size(theta_dot,2));
    
    if this_point >= size(theta_dot,2)
        this_point = size(theta_dot,2)-1;
        disp('????')
    end   
    
    all_this_point=[all_this_point this_point];

    if this_point > length(tvec)/(size(theta_points,2)-1)
        this_point
        break
    end
    
    
    
%     target_joint_position=theta(:,this_point).';
%     target_joint_position_next=theta(:,this_point+1).';
%   
%     target_joint_velocity=theta_dot(:,this_point);
%     target_joint_velocity_next=theta_dot(:,this_point+1);    

    target_joint_position=target_joint_position_next;
    target_joint_position_next=theta(:,this_point+1).';
    target_joint_velocity=target_joint_velocity_next;
    target_joint_velocity_next=theta_dot(:,this_point+1);  
    all_target_joint_position=[all_target_joint_position; target_joint_position;];

    this_p_7=init_pos;
    if i == 1
        this_p_7
        target_joint_position.'
        target_joint_position_next.'
    end

    all_jpos=[all_jpos this_p_7];  

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
  
   all_contact_force_after=[all_contact_force_after contact_force_after];

        v_cartesian_target=J_dx_dq*target_joint_velocity;
        all_v_cartesian_target=[all_v_cartesian_target v_cartesian_target];
     v_cartesian = J_dx_dq*feedback_joint_velocity_after;
        all_v_cartesian=[all_v_cartesian v_cartesian];

new_v_filt=[new_v_filt v_filt];
%% SVD
% [U,S,V] = svd(J_dx_dq)

%         max=30;   % 8  8 5
%         k=max/150; 
%         if i <=1000 && i>850
%             contact_force_after=[k*(i-850) 0 0].';
%         elseif i >1000 && i<=1900
%             contact_force_after=[max 0 0].';
%         elseif i >1900 && i<=2050
%             contact_force_after=[max-k*(i-1900) 0 0].';
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
     
    feedback_joint_velocity_after=pinv(Jac)*[v_filt; 0; 0; 0;];
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

if which_state_now == 11
    FINISH = 1
end

end




figure(1);plot(all_end_effector_p(1,:)); hold on; plot(all_target_end_effector_p(1,:));
legend('real','target')
figure(2);plot(all_end_effector_p(2,:)); hold on; plot(all_target_end_effector_p(2,:));
legend('real','target')
figure(3);plot(all_end_effector_p(3,:)); hold on; plot(all_target_end_effector_p(3,:));
legend('real','target')


figure(4);plot(all_jpos(1,:)); hold on; plot(all_target_joint_position(:,1));
legend('real','target')
figure(5);plot(all_jpos(2,:)); hold on; plot(all_target_joint_position(:,2));
legend('real','target')
figure(6);plot(all_jpos(4,:)); hold on; plot(all_target_joint_position(:,4));
legend('real','target')
figure(7);plot(all_jpos(6,:)); hold on; plot(all_target_joint_position(:,6));
legend('real','target')

% figure(52);
% 
% plot(adjust(1,:)); hold on;
% plot(all_acc(1,:)*100); hold on; 
% plot(new_v_filt(1,:)*300); hold on;
% plot(all_end_effector_p(1,:)*100-65);
% legend('filter force','acceleration','velocity','theortic position')
% 
% 
% refer_pos=refer_pos.';
% disp('Direct servo motion completed successfully')
% warning('on')
% figure(9);plot(refer_pos(:,1)); hold on; plot(all_end_effector_p(1,:));
% legend('goal','real')
% figure(10);plot(refer_pos(:,2)); hold on; plot(all_end_effector_p(2,:));
% legend('goal','real')
% figure(11);plot(refer_pos(:,3)); hold on; plot(all_end_effector_p(3,:));
% legend('goal','real')
% % figure(12);plot(all_dt_real)
% figure(13);plot(rate_target(1,:)); hold on; plot(rate_xdetjia(1,:)); hold on; plot(new_v_filt(1,:))
% figure(14);plot(rate_target(2,:)); hold on; plot(rate_xdetjia(2,:)); hold on; plot(new_v_filt(2,:))
% figure(15);plot(rate_target(3,:)); hold on; plot(rate_xdetjia(3,:)); hold on; plot(new_v_filt(3,:))


