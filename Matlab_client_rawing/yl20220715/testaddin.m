% This file is used to test new function, please run it, and vrep could be
% table 222 file. 有很多意义不明的初始化，不用管他，这个就是测试用的，想要啥新功能在循环里面测试

clear;
clc;
EX_force=[];EX_force_ori=[];KEY_CONTROL=0;count_loop=0;debug=[];
% Controller gain
Kp_joint = eye(7)*20;
Kp_cart = eye(3)*15;
% Redundancy gain
k0 = 10; %0 for standard (non collision-free) motion planning 
% Integration Time Step
Ts = 0.05;
% Obstacles
obs = [[-0.025;-0.55;0.8]];
% Time execution
T_tot = 20;
% Controller type
controller = "joint";
%% Simulation
robot = VrepConnector(19999,0.01);
% theta_points=[[0,0.0024*180/pi,0,-90,0,90,0].'];
 theta_points=[ -0.4013,0.9539,0,-1.0490,0,1.1374,-0.3927].';
hh=directKinematics(theta_points(:,1))
q = robot.GetState()
while(norm(directKinematics(q) - directKinematics(theta_points(:,1))) > 1 )
    norm(directKinematics(q) - directKinematics(theta_points(:,1)));
    now_2=directKinematics(q)
    q = robot.GetState();
end
now_3=directKinematics(q)
%% LOOP


% [way_points which_state_now]= RL2m3m3(robot);
% [useless, len_way_points]=size(way_points);
% 
% test_points=[now_3 way_points(:,1)]

qd = robot.GetState();

test_pos_table=[0.61792 0 0.591];

test_end_table=[0.42 0 0.7];
TABLE=[test_pos_table' test_end_table'];
all_pos=[];all_points=[];


%% Trajectory setup
% way_points = [[-0.4003;0;0.5690],[-0.025;-0.6;0.5],[0.425;-0.3;0.7],[-0.025;-0.55;1.1],[-0.4003;0;0.5690]];
% way_points = [[-0.525;-0.2;0.9],[-0.025;-0.6;0.5],[0.425;-0.3;0.7],[-0.025;-0.55;1.1],[-0.525;-0.2;0.9]];
% way_points = [[-0.4009;0.0000;0.7790],[-0.4503;-0.45;0.769],[0.425;-0.3;0.7],[-0.025;-0.55;1.1],[-0.4009;0.0000;0.7790]];

% theta_points=[q];
% % way_points = [[0.4009;0.1;0.549],[0.5;0.3;0.519],[0.475;0.25;0.049]];
%  theta_points=theta_points*180/pi;
% for i = 2 : length(way_points)
%     init_theta=0;
%     xd=way_points(1,i)
%     yd=way_points(2,i)
%     zd=way_points(3,i)
%     [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta).'
%     while length(All_theta) ==0
% %         init_theta=init_theta+1;
%         yd=yd-0.1;
%         [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta).';
%     end
% %     All_theta=All_theta+[180
%     [hang,lie]=size(All_theta);
%     temp=theta_points(:,end);
%     tott=1000;
%     delta_matrix=All_theta-temp;
%     for each_lie =1:lie
%         now=sum(abs(All_theta(:,each_lie)-temp))
% %          now=abs(sum(All_theta(:,each_lie)-temp))
% %         now=abs(sum(All_theta(1:end-1,each_lie)-temp(1:end-1,:)))
%         if now < tott;
%             tott=now;
%             which=each_lie;
%         end
%         
%     end
%     to_add=All_theta(:,which);
%     to_add(7)=0;
%     theta_points=[theta_points to_add]
% end
% theta_points=theta_points.*pi/180;
% 
% 
% 
% 
% theta_d_des=[];
% for z =1:len_way_points
%     theta_d_des=[theta_d_des [0;0;0;0;0;0;0]];
% %     theta_d_des = [[0;0;0;0;0;0;0],[0;0;0;0;0;0;0],[0;0;0;0;0;0;0],[0;0;0;0;0;0;0]];
% end
% tvec = 0:Ts:T_tot;
% tpts = 0:T_tot/(size(theta_points,2)-1):T_tot;
% 
% [theta,theta_dot,theta_dotdot,pp] = cubicpolytraj(theta_points,tpts,tvec,...
%                 'VelocityBoundaryCondition', theta_d_des);
            
% h=[];            
% for ww = 1:5            
%     now_q=theta_points(:,ww);
%     hh=directKinematics(now_q);
%     h=[h hh];
% end
% h


% plot(tvec, theta)
% hold all
% plot(tpts, theta_points, 'x')
% xlabel('t')
% ylabel('Positions')
% legend('X-positions','Y-positions')
% hold off
% %您还可以验证二维平面中的实际位置。将q向量和路径点的单独行绘制为x和y位置。
% figure
% plot(theta(1,:),theta(2,:),'-b',theta_points(1,:),theta_points(2,:),'or')
% xlabel('X')
% ylabel('Y')


%%  initialization
    % simulation period
    dt = 0.05;
    % simulation duration you can make it larger if you'd like to delete
    % the little stone in V-REP during the simulation


    %%  initialization 2
    noo=robot.Inital();
%     pause(.3);
    % get simulation time
    currentCmdTime = robot.GetLastCmdTime();
    lastCmdTime = currentCmdTime;
    % limits for joint position and velocity
    qmax = [170,120,170,120,170,120,175]*pi/180;
    qmin = -qmax;
    dqlimit = [110,110,128,128,204,184,184]*pi/180;
    % some variables for computation
    feedback_joint_position = zeros(7,1);
    feedback_joint_velocity = zeros(7,1);
    feedback_joint_torque = zeros(7,1);
    target_joint_position = zeros(7,1);
    target_joint_velocity = zeros(7,1);
%     external_force_vec = zeros(3,1);
%     external_torque_vec = zeros(3,1);
    contact_force = zeros(3,1);
    % get init state
    feedback_joint_position=robot.GetState()
    feedback_joint_velocity=robot.GetV()
    % velocity calculated by difference
%     feedback_joint_velocity = (feedback_joint_position - lastConfiguration)/dt;
    target_joint_position = feedback_joint_position;
    target_joint_velocity = feedback_joint_velocity;
    % store last and current configuration
    lastConfiguration = feedback_joint_position;
    contact_force0000 = robot.GetF()
    % get init contact force
%     while length(contact_force)==0
%      contact_force = robot.GetF()
%     end
    
    contact_force = contact_force';    %
    robot_type = 1;
    [ pose, nsparam, rconf, jout ] = ForwardKinematics( feedback_joint_position, robot_type );
    target_end_effector_p = pose(1:3,4);
    target_end_effector_r = pose(1:3,1:3);
    % some variables for data record and plot
    v_cartesians = [];
    p_cartesians = [];
%     f_external_forces = [];
    f_contact_forces = [];
    f_attractors = [];
    f_nets = [];
    q_ddots = [];
    target_joint_positions = [];
    target_joint_velocities = [];
    feedback_joint_ps = [];
    feedback_joint_vs = [];
    feedback_joint_ts = [];
    %% natural admittance control
    % cartesian impedance controller parameters
    k_cartesian = diag([100,100,80]);
    b_cartesian = diag([150,150,120]);
    % equivalent inertial in joint space
    H_inv = diag([1 1 1 1 1 1 0]);
    % gain on the orientation difference of the end effector
    k_vel_p = 50;
    
    % trigger simulation
    Inter=robot.Intergate();
   
    % time record
    time = [];
    t = 0;
round=0;first_roud=0;
while round < 4    
    round=round+1
    
    q = robot.GetState();
%     tic
    [total_act way_points which_state_now myspace]= RL2m3m3(robot);
%     toc
% break
[useless, len_way_points]=size(way_points);

    theta_points=[q];
% way_points = [[0.4009;0.1;0.549],[0.5;0.3;0.519],[0.475;0.25;0.049]];
 theta_points=theta_points*180/pi;
 
  [hang,lie]=size(way_points);
for i = 2 : lie
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
                    %                 now_y=now_y-0.1;
                
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
debug=[debug way_points];



theta_d_des=[];
for z =1:len_way_points
    theta_d_des=[theta_d_des [0;0;0;0;0;0;0]];
end

T_tot=20-count_loop*Ts;
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(theta_points,2)-1):T_tot;

[theta,theta_dot,theta_dotdot,pp] = cubicpolytraj(theta_points,tpts,tvec,...
                'VelocityBoundaryCondition', theta_d_des);
            
tvec = 0:0.1:1;
tpts = 0:1;         
[points3,points_dot3,points_dotdot3,pp3] = cubicpolytraj(TABLE,tpts,tvec,...
                'VelocityBoundaryCondition', zeros(3,2));   
    %% control
clear_loop=0;    
size(theta_dot,2)


for i=1:size(theta_dot,2)
    count_loop=count_loop+1;
    clear_loop=clear_loop+1;
    if clear_loop > 100
        if first_roud == 0
            first_roud=1;
        end
        break
    end
%% caculate outer force

    [ pose_c, nsparam_c, rconf_c, jout_c ] = ForwardKinematics( theta(:,i), robot_type );
    target_end_effector_p = pose_c(1:3,4);
    target_end_effector_r = pose_c(1:3,1:3);

% time update
        currentCmdTime = robot.GetLastCmdTime();
        dt = (currentCmdTime-lastCmdTime)/1000;
        % get states feedback
    feedback_joint_position=robot.GetState();
    feedback_joint_velocity=robot.GetV();
%         feedback_joint_velocity = (feedback_joint_position-lastConfiguration)/dt;
        lastConfiguration = feedback_joint_position;
%         velocity_errors = [velocity_errors,target_joint_velocity - feedback_joint_velocity];
        % get external force in force sensor frame
        GetFF=robot.GetF();
%         GetVV=robot.GetV()
%         GetSS=robot.GetState()

%         contact_force = zeros(3,1);
       contact_force = GetFF';
        
        EX_force_ori=[EX_force_ori; contact_force;];
%         % controller
        [Jac,A_mat_products] = Jacobian(feedback_joint_position,robot_type);
        J_dx_dq = Jac(1:3,:);
%         
        [ pose, nsparam, rconf, jout ] = ForwardKinematics( feedback_joint_position, robot_type );
        end_effector_p = pose(1:3,4);
        end_effector_r = pose(1:3,1:3);
        see_contact_force=contact_force.';
        see_pro_contact_force=end_effector_r*(contact_force);
        
        
%         f_bias=[10 2 15].';
        f_bias=[0 0 0].';
        f_bias=end_effector_r*f_bias;
%         projection=end_effector_r*contact_force';
        contact_force = contact_force'-f_bias;
        
        v_cartesian = J_dx_dq*feedback_joint_velocity;
        f_attractor = k_cartesian*(target_end_effector_p-end_effector_p)-b_cartesian*v_cartesian;
        % transform external force to cartesian frame
%         external_force_vec = A_mat_products{7}(1:3,1:3)*external_force_vec';
%         f_net = f_attractor + external_force_vec;
        f_net = (f_attractor + contact_force);
        J_transpose_f = J_dx_dq'*f_net;
        q_ddot = H_inv*(J_transpose_f-feedback_joint_velocity);
        target_joint_velocity = target_joint_velocity + q_ddot*dt;
%         target_joint_velocity = feedback_joint_velocity;
        [issingular,sols]=SolveSphericalWrist(feedback_joint_position,target_end_effector_r,robot_type);
%         if size(sols,2)>0
%             q_vec_err = sols(:,1) - feedback_joint_position;
%             if size(sols,2)>1
%                 q_err1 = norm(q_vec_err);
%                 q_err2 = norm(sols(:,2) - feedback_joint_position);
%                 if q_err2<q_err1
%                     q_vec_err = sols(:,2) - feedback_joint_position;
%                 end
%             end
%             for i=5:7
%                 target_joint_velocity(i) =  k_vel_p*q_vec_err(i);
%                 
%             end
%         end
% %         
        target_joint_velocity = satdq(dqlimit,target_joint_velocity);
    
        % send command
        lastCmdTime = currentCmdTime;
        currentCmdTime = robot.GetLastCmdTime();
        t = t+dt;


 %% Apply control
%     theta_dot(:,i)
%     target_joint_velocity

    pause(Ts);
% end_velocity=theta_dot(:,i)+target_joint_velocity/50;
    end_velocity=theta_dot(:,i);
%     end_velocity=[0;0;0;0;0;0;0];



%% PID addF

m_table=16.67; b_table=152.776; k_table=975;

    
    w_n=(k_table/m_table)^0.5;
    zeta=b_table/m_table/2/w_n;
    
    
pos = robot.position_of_table()
all_pos=[all_pos pos'];
velocity = robot.velocity_of_table()
a_e=-velocity(3)/dt;
v_e=-velocity(3);
if i < 11
    x_e=-(pos(3)-0.591);
else
    x_e=-(pos(3)-0.72);
end
F_e=m_table*a_e+b_table*v_e+k_table*x_e
F_sensor=robot.GetF()
robot.Applyhand2(0,0,F_e)

% force_x=0;
% force_y=0;
% if i <=4
%     force_z=10;
% elseif i<=8
%     
%     force_z=-10;
% else
%     force_z=0;
% end
% force_z
    

%     eul = [0 pi/2 pi];
%     tformZYZ = eul2tform(eul);
%     tformZYZ(1:3,4)=points3(:,i)
%     [x,y,z,w] = quat2tform(tformZYZ)


if i > size(points3,2)
    i = size(points3,2);
end
x=points3(1,i);
y=points3(2,i);
z=points3(3,i);
all_points=[all_points points3(:,i)];
%     robot.set_position_table(x,y,z)
    
%     robot.ApplyPosi(theta(:,i));
%     robot.ApplyControl(end_velocity, Ts);
    robot.ApplyControl(zeros(7,1), Ts);
    i
end            
end
% subplot(2,2,1)
% plot(EX_force_ori(1,:))
% subplot(2,2,2)
% plot(EX_force_ori(2,:))
% subplot(2,2,3)
% plot(EX_force_ori(3,:))

for jb = 1:3
figure(10+jb);
plot(all_pos(jb,:)); hold on;
plot(all_points(jb,:))
legend('real x','desired x')
end
