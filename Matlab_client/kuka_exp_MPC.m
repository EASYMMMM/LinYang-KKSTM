%% Example of using KST class for interfacing with KUKA iiwa robots
% This example script is used to show how to utilise
% the soft real time control of the KUKA iiwa 7 R 800 for
% Moving first joint of the robot, using a sinusoidal function

% First start the server on the KUKA iiwa controller
% Then run this script using Matlab

% Important: Be careful when runnning the script, be sure that no human, nor obstacles
% are around the robot

close all;clear;clc;
warning('off')
%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main')  

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
%% Start direct servo in joint space       
w=1.5; % motion constants, frequency rad/sec
A=pi/6; % motion constants, amplitude of motion
counter=0;
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];v_filt=[];new_v_filt=[];all_acc=[];all_init_direction_theory=[];init_direction_theory=[0.6 -0.3 0.25].';


%% Start direct servo in joint space       
iiwa.realTime_startVelControlJoints();all_output=[];
% iiwa.realTime_startImpedanceJoints(0.6,0,0,0.1,3500,250,10);

% iiwa.realTime_startVelControlJoints();

w=1.5; % motion constants, frequency rad/sec
A=pi/6; % motion constants, amplitude of motion
counter=0;
all_goal_theta=[];
pos_x=[];pos_y=[];pos_z=[];v_filt=[];new_v_filt=[];
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
    
    k_cartesian = diag([100,100,100]*1*1)*1.3*5/2/2;
    b_cartesian = diag([100,100,100]*14*0.707*45/1000*0.7*5*1.4/2*1.4/2);   
    H_inv = diag([1 1 1]/10/5*3*2*2);
    
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

init_direction=[0.6; -0.3; 0.25];
accum_dt=0;all_xe=[]; all_xde=[];

feedback_joint_velocity_after=zeros(7,1);all_output=[];
init_pos=[-0.463647609000806,0.820126730911849,0,-1.17997019621945,0,1.14149572645849,2.67794504458899].';
count_loop=0;
rate_xdetjia=[];rate_target=[];
target_joint_position_next=[-0.463647609000806,0.820126730911849,0,-1.17997019621945,0,1.14149572645849,2.67794504458899];
% target_joint_position_next=[-0.432399711794801;0.933500000015965;0;-0.967999999993481;0;1.24009999998811;0].';
% 0.6 -0.3 0.25
target_joint_position=target_joint_position_next;
FINISH=0;round=0; compared_p=[];compared_v=[];all_target_joint_position=[];all_real_joint_position=[];


target_joint_velocity_next=[0 0 0 0 0 0 0].';all_this_point=[];last_p=target_joint_position_next;all_xetjia=[];v_filt=[0 0 0].';
tic;
Ts=0.010;
dt_real=Ts;
while FINISH == 0
    round=round+1;
          
            this_p_7=init_pos;
            [ pose, nsparam, rconf, jout ] = ForwardKinematics( this_p_7, robot_type );
            now_pos_3=pose(1:3,4);           
            [total_act way_points which_state_now myspace]= RL2m3m3(now_pos_3);
            [useless, len_way_points]=size(way_points);
            
            theta_points=target_joint_position.';
           
            theta_points=theta_points*180/pi;
            
%   [new_way_points]= simplifywaypoints(way_points,total_act)
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


T_tot=(size(theta_points,2)-1)*15;
if T_tot == 0
    disp('arrive')
    break
end
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(theta_points,2)-1):T_tot;

[theta,theta_dot,theta_dotdot,pp] = cubicpolytraj(theta_points,tpts,tvec,...
                'VelocityBoundaryCondition', theta_d_des);

clear_loop=0;   accum_dt=0;

% refer_pos=[];
% for q=1:size(theta_dot,2)
%          [ pose, nsparam, rconf, jout ] = ForwardKinematics( theta(:,q).', robot_type );
%         end_effector_p = pose(1:3,4);
%         refer_pos=[refer_pos end_effector_p];
% end


for i=1:3000
    i;
    if i == 1
        start = toc
    end
    accum_dt=accum_dt+dt_real;
    all_dt_real=[all_dt_real dt_real];
    dt=dt_real;
    count_loop=count_loop+1;
    clear_loop=clear_loop+1;
    
% 
%         direction=which_refer_direction(total_act)
%         [y,iy]=min(abs(refer_pos(direction,:)-now_pos_3(direction)));
%     %     ratio=(init_direction-init_direction_0)/(init_direction_end-init_direction_0);
        this_point=ceil(accum_dt/T_tot*size(theta_dot,2));
%         
%         real_point=iy;
%         if real_point >= size(theta_dot,2)
%             real_point = size(theta_dot,2)-1;
%         end
%         if this_point >= size(theta_dot,2)
%             this_point = size(theta_dot,2)-1;
%         end
%         all_this_point=[all_this_point this_point];
%     
%     if i < 20
%         this_point=real_point;
%     end
    
    
    
    if this_point >= size(theta_dot,2)
        this_point = size(theta_dot,2)-1;
        disp('????')
    end   
    
    if this_point > length(tvec)/(size(theta_points,2)-1)
        this_point
        this_p_break=this_p
        break
    end
    
    all_this_point=[all_this_point this_point];
    
    target_joint_position=target_joint_position_next;
    target_joint_position_next=theta(:,this_point+1).';
    target_joint_velocity=target_joint_velocity_next;
    target_joint_velocity_next=theta_dot(:,this_point+1);  
    all_target_joint_position=[all_target_joint_position; target_joint_position;];
    this_p=iiwa.getJointsPos();
    this_p=cell2mat(this_p);
    
    all_jpos=[all_jpos; this_p;];  
    
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
    feedback_joint_position=this_p;
   
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



         [ pose, nsparam, rconf, jout ] = ForwardKinematics( feedback_joint_position, robot_type );
        end_effector_p = pose(1:3,4);
        all_end_effector_p=[all_end_effector_p end_effector_p];
        
        end_effector_r = pose(1:3,1:3);


        f_bias=1*[0 0 0].';
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
%        contact_force_after=[0 0 0].';



        v_cartesian_target=J_dx_dq*target_joint_velocity;
        all_v_cartesian_target=[all_v_cartesian_target v_cartesian_target];
   
        
        v_cartesian = J_dx_dq*feedback_joint_velocity_after;
        all_v_cartesian=[all_v_cartesian v_cartesian];

  
% v_filt=[v_filtered1 v_filtered2 v_filtered3].';
new_v_filt=[new_v_filt v_filt];




%% SVD
% [U,S,V] = svd(J_dx_dq)

%         max=5;   % 8  8 5
%         k=max/150; 
%         if i <=1000 && i>850
%             contact_force_after=[0 0 k*(i-850)].';
%         elseif i >1000 && i<=1500
%             contact_force_after=[0 0 max].';
%         elseif i >1500 && i<=1650
%             contact_force_after=[0 0 max-k*(i-1500)].';
% %             contact_force_after=[0 0 0].';
%         end
% %         contact_force_after=[8 0 0].';
%         adjust=[adjust contact_force_after];        

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
     %% SAFE
    future_pos_7=this_p.'+target_joint_velocity_next*dt;
    for each_joint = 1:7
        if (future_pos_7(each_joint) <= qmin(each_joint)) || (future_pos_7(each_joint) >= qmax(each_joint))
            disp('ERROR ! range is out of limit')
            feedback_joint_velocity_after(each_joint)=0;
        end
    end
    future_pos_3=end_effector_p+J_dx_dq*target_joint_velocity_next*dt;
    if future_pos_3(3)<0.05
        disp('ERROR ! range is out of limit')
        break
    end

    all_feedback_joint_velocity_after=[all_feedback_joint_velocity_after feedback_joint_velocity_after];

    this_theta_matrix=target_joint_velocity_next.';
    this_theta=num2cell(this_theta_matrix);

    this_zukang=num2cell(feedback_joint_velocity_after);

    if i < 20
        my_t=iiwa.sendJointsVelocitiesExTorques(this_zukang);
    else 
        my_t=iiwa.sendJointsVelocitiesExTorques(this_zukang);
    end

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
if which_state_now == 11
    FINISH = 1
end

end

figure(59);
plot(new_f(3,:)); hold on;
plot(all_contact_force_after(3,:)); hold on;
plot(all_F_contact(3,:)); hold on; 
plot(all_acc(3,:)*100); hold on; 
plot(new_v_filt(3,:)*300); hold on;
plot(all_init_direction_theory(3,:)*100-20); hold on; 
plot(all_end_effector_p(3,:)*100-20);
legend('KKKK','filter force','Force','acceleration','velocity','theortic position','real position')


% plot(all_contact_force_after(1,:)); hold on;plot(new_f(1,:)); hold on; plot(all_end_effector_p(1,:)*100-65);hold on; plot(new_v_filt(1,:)*100); 
figure(60)
plot(all_contact_force_after(3,:)); hold on;
plot(-all_f_attractor(end,:))
legend('filter force','following force')


%% turn off server
% iiwa.realTime_stopImpedanceJoints()
iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer()


disp('Direct servo motion completed successfully')
warning('on')

figure(9);plot(all_end_effector_p(1,:)); hold on; plot(all_target_end_effector_p(1,:));
legend('real','goal')
figure(10);plot(all_end_effector_p(2,:)); hold on; plot(all_target_end_effector_p(2,:));
legend('real','goal')
figure(11);plot(all_end_effector_p(3,:)); hold on; plot(all_target_end_effector_p(3,:));
legend('real','goal')

% figure(13);plot(rate_target(1,:)); hold on; plot(rate_xdetjia(1,:)); hold on; plot(new_v_filt(1,:))
% figure(14);plot(rate_target(2,:)); hold on; plot(rate_xdetjia(2,:)); hold on; plot(new_v_filt(2,:))
% figure(15);plot(rate_target(3,:)); hold on; plot(rate_xdetjia(3,:)); hold on; plot(new_v_filt(3,:))
all_jpos=all_jpos.';
figure(4);plot(all_jpos(1,:)); hold on; plot(all_target_joint_position(:,1));
legend('real','target')
figure(5);plot(all_jpos(2,:)); hold on; plot(all_target_joint_position(:,2));
legend('real','target')
figure(6);plot(all_jpos(4,:)); hold on; plot(all_target_joint_position(:,4));
legend('real','target')
figure(7);plot(all_jpos(6,:)); hold on; plot(all_target_joint_position(:,6));
legend('real','target')

figure(17);plot(all_dt_real)

% legend('goal','real')
%          clust = parcluster('local');
%          job1 = createJob(clust); %开启一个job
%          disp('saving------');
%          temp = all_v_cartesian.';
%          createTask(job1,@mytxt,1,{temp});%再给job1分配一个‘mytxt’的task
%          submit(job1);
% figure;plot(all_target_end_effector_p(1,:));hold on;
% figure;plot(all_target_end_effector_p(1,:)-all_end_effector_p(1,:))


