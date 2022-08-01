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
arg1=KST.LBR7R800; % choose the robot iiwa7R800 or iiwa14R820
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
theta_points=[ -0.4013,0.9539,0,-1.0490,0,1.1374,-0.3927];

theta_points2=num2cell(theta_points);
iiwa.movePTPJointSpace(theta_points2, relVel); % move to initial configuration


%% Caculate 

theta_points=[ -0.4013,0.9539,0,-1.0490,0,1.1374,-0.3927].';
way_points=[0.65;0.3;0.25];
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

Ts=0.01;
T_tot=20;
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(theta_points,2)-1):T_tot;

[theta,theta_dot,theta_dotdot,pp] = cubicpolytraj(theta_points,tpts,tvec,...
                'VelocityBoundaryCondition', theta_d_des);

%% Start direct servo in joint space       
iiwa.realTime_startDirectServoJoints();
% iiwa.realTime_startImpedanceJoints(0.6,0,0,0.1,3500,250,10);

% iiwa.realTime_startVelControlJoints();

w=1.5; % motion constants, frequency rad/sec
A=pi/6; % motion constants, amplitude of motion
counter=0;


%% Initiate timing variables
dt=0;
tic;
t0=toc; % calculate initial time

    %% natural admittance control
    % cartesian impedance controller parameters
    k_cartesian = diag([100,100,80]);
    b_cartesian = diag([150,150,120]);
    
    k_cartesian_7 = diag([100,100,80,100,100,100,50]/1);
    b_cartesian_7 = diag([150,150,120,100,100,50,80]/1);    
    M_1=diag([1 1 1]);
    
    % equivalent inertial in joint space
    H_inv = diag([1 1 1 1 1 1 0]*100);
    % gain on the orientation difference of the end effector
    k_vel_p = 50;
    % time record
    qmax = [170,120,170,120,170,120,175]*pi/180;
    qmin = -qmax;
    dqlimit = [110,110,128,128,204,184,184]*pi/180;
    t = 0;
    list_x=[];list_v=[];list_a=[];
   %%
   coef_v_order9=[
    0.09637703954237,  0.09877664875597,   0.1005959487483,   0.1018182036649,...
     0.1024321592885,   0.1024321592885,   0.1018182036649,   0.1005959487483,...
    0.09877664875597,  0.09637703954237
].';


  coef_f_order26=[
    0.02718719168633,  0.02960040617057,   0.0319283543919,  0.03414443214987,...
    0.03622288837122,  0.03813922211286,  0.03987056397838,  0.04139603507008,...
    0.04269707696676,  0.04375774671473,  0.04456497143587,  0.04510875788065,...
    0.04538235307079,  0.04538235307079,  0.04510875788065,  0.04456497143587,...
    0.04375774671473,  0.04269707696676,  0.04139603507008,  0.03987056397838,...
    0.03813922211286,  0.03622288837122,  0.03414443214987,   0.0319283543919,...
    0.02960040617057,  0.02718719168633
].';

             new_f=[];new_v=[];new=[];
    all_feedback_joint_position=[];
%% Control loop   
all_v_cartesian_target=[];all_v_cartesian=[];alldt=[];all_contact_force_after=[];
last_p=[-0.4013,0.9539,0,-1.0490,0,1.1374,-0.3927];all_target_joint_position_e=[];
All_v=[];EX_force_ori=[];all_F_contact=[];my_t=[{0} {0} {0} {0} {0} {0} {0}];all_f_attractor=[];all_end_effector_p=[];
robot_type=1;  all_jpos=[]; all_jtor=[];my_torque=[0 0 0 0 0 0 0];F_contact=0;all_joint_pos=[];all_target_end_effector_p=[];

q_t1dd=theta_dotdot(:,1);q_t1d=[0 0 0 0 0 0 0].';q_t1=[0 0 0 0 0 0 0].';intergal_goal=[];
all_pp=[];

        
for i=1:size(theta_dot,2)-1
    target_joint_position=theta(:,i).';
    target_joint_position_next=theta(:,i+1).';
    target_joint_dotdot=theta_dotdot(:,i);
    target_joint_velocity=theta_dot(:,i);
%     iiwa.realTime_startImpedanceJoints(0.6,0,0,0.1,3500,250,10);
    time=toc;
    dt=time-t0;
%     alldt=[alldt dt];
    dt=Ts;
    t0=toc;
    counter=counter+1;
    
    this_p=iiwa.getJointsPos();
    
    

    all_jpos=[all_jpos; this_p;];
    
    
    this_p=cell2mat(this_p);
    velocity=(this_p-last_p)/dt;
    All_v=[All_v; velocity;];
    last_p=this_p;
    

%% caculate outer force

    [ pose_c, nsparam_c, rconf_c, jout_c ] = ForwardKinematics( theta(:,i), robot_type );
    target_end_effector_p = pose_c(1:3,4);
    target_end_effector_r = pose_c(1:3,1:3);
    all_target_end_effector_p=[all_target_end_effector_p target_end_effector_p];  %-----------
    
    [ pose_c_next, nsparam_c_next, rconf_c_next, jout_c_next ] = ForwardKinematics( theta(:,i+1), robot_type );
    target_end_effector_p_next = pose_c_next(1:3,4);   
    
% time update

        % get states feedback
    feedback_joint_position=this_p;
%     feedback_joint_velocity=velocity.';
%         feedback_joint_velocity = (feedback_joint_position-lastConfiguration)/dt;
        lastConfiguration = feedback_joint_position;
        all_feedback_joint_position=[all_feedback_joint_position; feedback_joint_position;];
%         velocity_errors = [velocity_errors,target_joint_velocity - feedback_joint_velocity];
        % get external force in force sensor frame


    
    
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
new=[new; cell2mat(my_t);];
    

    feedback_joint_velocity=velocity.';
%     new_v=[new_v; velocity;];
    




    F_contact=1*pinv(J_dx_dq.')*my_torque;
    all_F_contact=[all_F_contact F_contact];       %%!!! 
        
        
         
 if i >= 26
        window_f = [new_f(:,end-24:end) F_contact];
        tempp_f = window_f*coef_f_order26;
        contact_force_after=tempp_f;
        new_f = [new_f contact_force_after];
 else
     contact_force_after=F_contact;
      new_f=[new_f contact_force_after];
 end     
    
  
         [ pose, nsparam, rconf, jout ] = ForwardKinematics( feedback_joint_position, robot_type );
        end_effector_p = pose(1:3,4);
        all_end_effector_p=[all_end_effector_p end_effector_p];
        
        end_effector_r = pose(1:3,1:3);
        
 %         f_bias=[10 2 15].';
        f_bias=1*[0 0 6].';
        f_bias=end_effector_r*f_bias;
%         projection=end_effector_r*contact_force';
        contact_force_after = (contact_force_after-f_bias);
 flagg=0;      
 thre=2;
 if abs(contact_force_after(1))<thre
     contact_force_after(1)=0;
     flagg=flagg+1;
 elseif contact_force_after(1)>=thre
     contact_force_after(1)=contact_force_after(1)-thre;
 elseif contact_force_after(1)<=-thre
     contact_force_after(1)=contact_force_after(1)+thre;
 end
 
  if abs(contact_force_after(2))<thre
     contact_force_after(2)=0;
     flagg=flagg+1;
 elseif contact_force_after(2)>=thre
     contact_force_after(2)=contact_force_after(2)-thre;
 elseif contact_force_after(2)<=-thre
     contact_force_after(2)=contact_force_after(2)+thre;
  end
 
  if abs(contact_force_after(3))<thre
     contact_force_after(3)=0;
     flagg=flagg+1;
 elseif contact_force_after(3)>=thre
     contact_force_after(3)=contact_force_after(3)-thre;
 elseif contact_force_after(3)<=-thre
     contact_force_after(3)=contact_force_after(3)+thre;
 end
    
   all_contact_force_after=[all_contact_force_after contact_force_after];
   



        v_cartesian_target=J_dx_dq*target_joint_velocity;
        all_v_cartesian_target=[all_v_cartesian_target v_cartesian_target];
        
        
        
        v_cartesian = J_dx_dq*feedback_joint_velocity;
        all_v_cartesian=[all_v_cartesian v_cartesian];
        
 if i >= 10
        window_v = [new_v(:,end-8:end) v_cartesian];
        tempp_v = window_v*coef_v_order9;
        v_cartesian=tempp_v;
        new_v = [new_v tempp_v];
  else
    
      new_v=[new_v v_cartesian];
  end  
        

        
        
        
        
        f_attractor = k_cartesian*(end_effector_p-target_end_effector_p+J_dx_dq*target_joint_velocity*dt)...
        +b_cartesian*(-v_cartesian_target+v_cartesian+J_dx_dq*target_joint_dotdot*dt);
        
        
        
        all_f_attractor=[all_f_attractor f_attractor];
%%
        f_net = (-f_attractor + contact_force_after);
        aa3=M_1*f_net;
        vv3=(v_cartesian-J_dx_dq*target_joint_velocity)+aa3*dt;
        pp3=(end_effector_p-target_end_effector_p)+vv3*dt;
        pp_plus=pp3+target_end_effector_p_next;
        all_pp=[all_pp pp3];

        
%%
        q_t1dd=H_inv*(J_dx_dq'*contact_force_after-k_cartesian_7*q_t1-b_cartesian_7*q_t1d);
        list_a=[list_a q_t1dd*dt];
        q_t1d=sum(list_a,2);
        list_v=[list_v q_t1d*dt];
        q_t1=sum(list_v,2);
        list_x=[list_x q_t1*dt];  
        goal=target_joint_position_next+q_t1*dt;
        intergal_goal=[intergal_goal; goal;]; 
   %%      
        J_transpose_f = J_dx_dq'*f_net;
        q_ddot = H_inv*(J_transpose_f);       
        
        feedback_joint_velocity=pinv(J_dx_dq)*v_cartesian;
    
        
        target_joint_velocity_plus = (feedback_joint_velocity-target_joint_velocity) + q_ddot*dt;
%         target_joint_velocity_plus = q_ddot*dt;



        target_joint_velocity3=target_joint_velocity_plus.';
        target_joint_position_e = (feedback_joint_position-target_joint_position) + target_joint_velocity3*dt;
        
    
%         if max(target_joint_position_e)<0.01
%             target_joint_position_e=[0 0 0 0 0 0 0];
%         end
        
        all_target_joint_position_e=[all_target_joint_position_e; target_joint_position_e;];
      
        final_target_joint_position=target_joint_position_next+target_joint_position_e;
        
        final_target_joint_position2 = satq(qmin,qmax,final_target_joint_position);
        all_joint_pos=[all_joint_pos; final_target_joint_position2;];
        t = t+dt;








    this_theta_matrix=theta(:,i).';
    this_theta=num2cell(this_theta_matrix);
%     iiwa.sendJointsPositions(this_theta);
    this_zukang=num2cell(final_target_joint_position2);
    
    
    if i < 26 
        my_t=iiwa.sendJointsPositionsExTorque(this_theta);
    elseif flagg==3
        my_t=iiwa.sendJointsPositionsExTorque(this_theta);
    else 
        my_t=iiwa.sendJointsPositionsExTorque(this_zukang);
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


%          clust = parcluster('local');
%          job1 = createJob(clust); %开启一个job
%          disp('saving------');
%          temp = all_v_cartesian.';
%          createTask(job1,@mytxt,1,{temp});%再给job1分配一个‘mytxt’的task
%          submit(job1);




