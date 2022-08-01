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
theta_points=[ -0.4013,0.9539,0,-1.0490,0,1.1374,0];
[-0.401300000000000;0.953900000000000;0;-1.04900000000000;0;1.13740000000000;0]
[-0.405703173982151,0.956061280776520,0,-1.04480238839663,0,1.13946500390074,0]

theta_points2=num2cell(theta_points);all_gamma=[];all_beta=[];all_alpha=[];
iiwa.movePTPJointSpace(theta_points2, relVel); % move to initial configuration

%% Kalman
R = 0.05;  % 测量噪音方差矩阵
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
%% Caculate 

theta_points=[ -0.4013,0.9539,0,-1.0490,0,1.1374,0].';sum_e=0;
[-0.401300000000000;0.953900000000000;0;-1.04900000000000;0;1.13740000000000;0]
way_points=[0.65;0.3;0.25];theta_points_final=[];
[useless, len_way_points]=size(way_points);
 theta_points=theta_points*180/pi;
 
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
%     theta_d_des = [[0;0;0;0;0;0;0],[0;0;0;0;0;0;0],[0;0;0;0;0;0;0],[0;0;0;0;0;0;0]];
end

Ts=0.010;
T_tot=30;
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(theta_points,2)-1):T_tot;

[theta,theta_dot,theta_dotdot,pp] = cubicpolytraj(theta_points,tpts,tvec,...
                'VelocityBoundaryCondition', theta_d_des);

%% Start direct servo in joint space       
iiwa.realTime_startDirectServoJoints();all_output=[];
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
k2=1.5;
k3=1;

    k_cartesian = diag([100*k1,100*k2,100*k3]*2*5);
    b_cartesian = diag([100,100,100]*14*0.707*15/1000);   
    H_inv = diag([1*k1 1*2 1*k3]*100*0.1/2);
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
   %%
   coef_v_order9=[
    0.09637703954237,  0.09877664875597,   0.1005959487483,   0.1018182036649,...
     0.1024321592885,   0.1024321592885,   0.1018182036649,   0.1005959487483,...
    0.09877664875597,  0.09637703954237
].';
all_target_end_effector_p_next=[];

  coef_f_order26=[
    0.02718719168633,  0.02960040617057,   0.0319283543919,  0.03414443214987,...
    0.03622288837122,  0.03813922211286,  0.03987056397838,  0.04139603507008,...
    0.04269707696676,  0.04375774671473,  0.04456497143587,  0.04510875788065,...
    0.04538235307079,  0.04538235307079,  0.04510875788065,  0.04456497143587,...
    0.04375774671473,  0.04269707696676,  0.04139603507008,  0.03987056397838,...
    0.03813922211286,  0.03622288837122,  0.03414443214987,   0.0319283543919,...
    0.02960040617057,  0.02718719168633
].';

             new_f=[];new_v=[];new_torque=[];
    all_feedback_joint_position=[];
%% Control loop   
all_v_cartesian_target=[];all_v_cartesian=[];alldt=[];all_contact_force_after=[];
all_target_joint_position_e=[];
All_v=[];EX_force_ori=[];all_F_contact=[];my_t=[{0} {0} {0} {0} {0} {0} {0}];all_f_attractor=[];all_end_effector_p=[];
robot_type=1;  all_jpos=[]; all_jtor=[];my_torque=[0 0 0 0 0 0 0];F_contact=0;all_joint_pos=[];all_target_end_effector_p=[];
all_dt_real=[];
q_t1dd=theta_dotdot(:,1);q_t1d=[0 0 0 0 0 0 0].';q_t1=[0 0 0 0 0 0 0].';intergal_goal=[];adjust=[];
pause(3);accum_dt=0;all_xe=[]; all_xde=[];

tic;
t0=toc-Ts; % calculate initial time
target_joint_position_next=[-0.401396453114820,0.954044131099899,7.47815770448098e-05,-1.04905153079427,4.89915666364060e-05,1.13728358894867,-6.17187693413365e-05];
target_joint_velocity_next=[0 0 0 0 0 0 0].';all_this_point=[];last_p=target_joint_position_next;all_xetjia=[];
for i=1:size(theta_dot,2)
%     target_joint_position=theta(:,1).';
%     target_joint_position_next=theta(:,1).';
%     target_joint_dotdot=theta_dotdot(:,1);
%     target_joint_velocity=theta_dot(:,1);
%     target_joint_velocity_next=theta_dot(:,1);
    

    time=toc;
    dt_real=time-t0;
    accum_dt=accum_dt+dt_real;
    all_dt_real=[all_dt_real dt_real];
    dt=dt_real;
    t0=toc;
    counter=counter+1;
    
    
    this_point=ceil(accum_dt/T_tot*size(theta_dot,2));
    if this_point > size(theta_dot,2)
        this_point = size(theta_dot,2);
    end
    all_this_point=[all_this_point this_point];
    
%     target_joint_position=theta(:,this_point).';
%     target_joint_position_next=theta(:,this_point+1).';
%   
%     target_joint_velocity=theta_dot(:,this_point);
%     target_joint_velocity_next=theta_dot(:,this_point+1);    
    
    target_joint_position=target_joint_position_next;
    target_joint_position_next=theta(:,this_point).';
 
    target_joint_velocity=target_joint_velocity_next;
    target_joint_velocity_next=theta_dot(:,this_point);       
    
    
    
    
    
    
    
    
    this_p=iiwa.getJointsPos();
    
    

    all_jpos=[all_jpos; this_p;];
    
    
    this_p=cell2mat(this_p);
    velocity=(this_p-last_p)/dt;
    All_v=[All_v; velocity;];
    last_p=this_p;
    

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
        lastConfiguration = feedback_joint_position;
        all_feedback_joint_position=[all_feedback_joint_position; feedback_joint_position;];
        contact_force = F_contact;
%         % controller
        [Jac,A_mat_products] = Jacobian(feedback_joint_position,robot_type);
        J_dx_dq = Jac(1:3,:);
        
%     my_torque=cell2mat(my_t).';
    
    

%   if i >= 10
%         window = [new(i - 9 : i - 1,:); cell2mat(my_t)];
%         tempp = mean(window);
%         my_torque=tempp.';
%         new = [new; tempp];
%   else
%       my_torque=cell2mat(my_t).';
%       new=[new; cell2mat(my_t);];
%       
%   end
my_torque=cell2mat(my_t).';
new_torque=[new_torque; cell2mat(my_t);];
    

    feedback_joint_velocity=velocity.';
%     new_v=[new_v; velocity;];
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
%         sb=end_effector_r(3,1);
%         beta=asind(-sb);
%         cacb=end_effector_r(1,1);
%         alpha=acosd(cacb/cosd(beta));
%         cbsy=end_effector_r(3,2);
%         gamma=asind(cbsy/cosd(beta));
%         all_gamma=[all_gamma gamma];
%         all_beta=[all_beta beta];
%         all_alpha=[all_alpha alpha];
        
        
 %         f_bias=[10 2 15].';
        f_bias=1*[0 0 0].';
        f_bias=end_effector_r*f_bias;
%         projection=end_effector_r*contact_force';
        contact_force_after = (F_filt-f_bias);
 flagg=0;      
 thre=3;
 if abs(contact_force_after(1))<thre
     contact_force_after(1)=0;
     flagg=flagg+1;
 elseif contact_force_after(1)>=thre
     contact_force_after(1)=contact_force_after(1)-thre;
     contact_force_after(1)=contact_force_after(1)/3;
 elseif contact_force_after(1)<=-thre
     contact_force_after(1)=contact_force_after(1)+thre;
     contact_force_after(1)=contact_force_after(1)/3;
 end
 
  if abs(contact_force_after(2))<thre
     contact_force_after(2)=0;
     flagg=flagg+1;
 elseif contact_force_after(2)>=thre
     contact_force_after(2)=contact_force_after(2)-thre;
     contact_force_after(2)=contact_force_after(2)/3;
 elseif contact_force_after(2)<=-thre
     contact_force_after(2)=contact_force_after(2)+thre;
     contact_force_after(2)=contact_force_after(2)/3;
  end
 
  if abs(contact_force_after(3))<thre
     contact_force_after(3)=0;
     flagg=flagg+1;
 elseif contact_force_after(3)>=thre
     contact_force_after(3)=contact_force_after(3)-thre;
     contact_force_after(3)=contact_force_after(3)/3;
 elseif contact_force_after(3)<=-thre
     contact_force_after(3)=contact_force_after(3)+thre;
     contact_force_after(3)=contact_force_after(3)/3;
  end
    
 if i <10
     contact_force_after(3)=0;
 end
  
 thre_2=4;
  if contact_force_after(3)>thre_2
      contact_force_after(3)=thre_2;
  elseif contact_force_after(3)<-thre_2
       contact_force_after(3)=-thre_2;
  end
  if contact_force_after(1)>thre_2
      contact_force_after(1)=thre_2;
  elseif contact_force_after(1)<-thre_2
       contact_force_after(1)=-thre_2;
  end 
   if contact_force_after(2)>thre_2
      contact_force_after(2)=thre_2;
  elseif contact_force_after(2)<-thre_2
       contact_force_after(2)=-thre_2;
   end
  
   all_contact_force_after=[all_contact_force_after contact_force_after];
   



        v_cartesian_target=J_dx_dq*target_joint_velocity;
        all_v_cartesian_target=[all_v_cartesian_target v_cartesian_target];
   
        
        v_cartesian = J_dx_dq*feedback_joint_velocity;
        all_v_cartesian=[all_v_cartesian v_cartesian];
        
v_filtered1 = kalman1(v_cartesian(1));
v_filtered2 = kalman2(v_cartesian(2));
v_filtered3 = kalman3(v_cartesian(3));
  
v_filt=[v_filtered1 v_filtered2 v_filtered3].';
new_v_filt=[new_v_filt v_filt];

%  if i >= 10
%         window_v = [new_v(:,end-8:end) v_cartesian];
%         tempp_v = window_v*coef_v_order9;
%         v_cartesian=tempp_v;
%         new_v = [new_v tempp_v];
%   else
%     
%       new_v=[new_v v_cartesian];
%   end  
        
feedback_joint_velocity_after=pinv(J_dx_dq)*v_filt;

%% SVD
% [U,S,V] = svd(J_dx_dq)

%         max=5;   % 8  8 5
%         k=max/50; 
%         if i <=1500 && i>1450
%             contact_force_after=[0 0 k*(i-1450)].';
%         elseif i >1500 && i<=1900
%             contact_force_after=[0 0 max].';
%         elseif i >1900 && i<=1950
%             contact_force_after=[0 0 max-k*(i-1900)].';
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
%         equal_F=(-k_cartesian*xe-b_cartesian*xde);
%         all_f_attractor=[all_f_attractor equal_F];
     xdetjia=-J_dx_dq*target_joint_velocity+v_filt+x_t1dd*dt;
     
     xetjia=-target_end_effector_p+end_effector_p+xdetjia*dt;
     all_xetjia=[all_xetjia xetjia];

        
        goal=(target_end_effector_p_next+xetjia).'; %3 1-> 13
        
%         goal=target_end_effector_p_next.';
        intergal_goal=[intergal_goal; goal;];
        
        

  %% PD
  sum_e=sum_e+end_effector_p-goal.';
        output=P_coe*(end_effector_p-goal.')...
            +I_coe*(sum_e)...
            +D_coe*(v_filt-v_cartesian_target);
            
 %3 1
output=-output+goal.';
all_output=[all_output output];
%%        
    init_theta1=180;
    init_theta2=0;
    
%     xd=goal(1);
%     yd=output(2);
%     zd=output(3);
    
    xd=goal(1);
    yd=goal(2);
    zd=goal(3);
    
    [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
    count_no=0;
            while isempty(All_theta)
                    if yd<0
                        init_theta2=init_theta2+1;
%                         init_theta1=180-count_no;
                    elseif yd>0
                        init_theta2=init_theta2+1;
%                         init_theta1=180+count_no;
                    else
                        init_theta2=init_theta2+1;
%                         init_theta1=180+count_no;
                    end
                count_no=count_no+1;
                [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
            end
    [hang,lie]=size(All_theta);
    temp=(target_joint_position.')*180/pi;
    tott=1000;
    for each_lie =1:lie
        this=All_theta(:,each_lie);
        this(7)=0;
        now=sum(abs(this-temp));
        
        if now < tott;
            tott=now;
            which=each_lie;
        end
        
    end
    to_add=All_theta(:,which);
    to_add(7)=0;
    to_add=to_add*pi/180;
    theta_points_final=[theta_points_final to_add];    
 %% 
   
        goal2 = satq(qmin,qmax,to_add.');
        all_goal_theta=[all_goal_theta; goal2;];
        t = t+dt;



         [ pose_end, nsparam_end, rconf_end, jout_end ] = ForwardKinematics( goal2, robot_type );
        end_effector_p_end = pose_end(1:3,4);
        pos_x=[pos_x end_effector_p_end];



    this_theta_matrix=target_joint_position;
    this_theta=num2cell(this_theta_matrix);

    this_zukang=num2cell(goal2);
    
    all_flagg=[all_flagg flagg];
    if i < 60
        my_t=iiwa.sendJointsPositionsExTorque(this_theta);
%     elseif flagg==3
%         my_t=iiwa.sendJointsPositionsExTorque(this_theta);
    else 
        my_t=iiwa.sendJointsPositionsExTorque(this_theta);
    end
    
    all_jtor=[all_jtor; my_t;];

    
%     iiwa.realTime_stopImpedanceJoints()
end



tstart=t0;
tend=time;
rate=counter/(tend-tstart);
%% Stop the direct servo motion


% iiwa.realTime_stopVelControlJoints();


fprintf('\nTotal execution time is %f: \n',tend-t0 );
fprintf('\nThe rate of joint nagles update per second is: \n');
disp(rate);
fprintf('\n')
pause(2);
%% turn off server
% iiwa.realTime_stopImpedanceJoints()
iiwa.realTime_stopDirectServoJoints();
iiwa.net_turnOffServer()

disp('Direct servo motion completed successfully')
warning('on')
figure(9);plot(intergal_goal(:,1)); hold on; plot(all_end_effector_p(1,:));
legend('goal','real')
figure(10);plot(intergal_goal(:,2)); hold on; plot(all_end_effector_p(2,:));
legend('goal','real')
figure(11);plot(intergal_goal(:,3)); hold on; plot(all_end_effector_p(3,:));
legend('goal','real')
figure;plot(all_dt_real)
% figure(12);plot(theta_points_final(2,:)); hold on; plot(all_feedback_joint_position(:,2))
% legend('goal','real')
%          clust = parcluster('local');
%          job1 = createJob(clust); %开启一个job
%          disp('saving------');
%          temp = all_v_cartesian.';
%          createTask(job1,@mytxt,1,{temp});%再给job1分配一个‘mytxt’的task
%          submit(job1);
% figure;plot(all_target_end_effector_p(1,:));hold on;
% figure;plot(all_target_end_effector_p(1,:)-all_end_effector_p(1,:))


