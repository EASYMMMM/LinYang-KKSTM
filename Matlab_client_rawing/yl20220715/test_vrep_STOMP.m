clear;
clc;close all;
% I prefer to use this file to test my trajotory in 0428
%% Parameters
% Controller gain
Kp_joint = eye(7)*20;
Kp_cart = eye(3)*15;
% Redundancy gain
k0 = 10; %0 for standard (non collision-free) motion planning 
% Integration Time Step
Ts = 0.05;
% Time execution
T_tot = 20;
% Controller type
controller = "joint";
%% Simulation
robot = VrepConnector(19999,0.05);
addpath('C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client_rawing\EMG_IMU')
addpath 'C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main'
addpath 'C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client'
addpath 'C:\Lin YANG\from me\KUKA\KUKA_Matlab\TRO\STOMP_done\Intro-to-Robo-Proj-master'

            CHANGE=0;
            now_pos_3=[0.1 -0.45 0.2]';
            [lastq,lastR,total_act way_points which_state_now myspace cartis_obs]= RL2m3m3_maze(now_pos_3,0,CHANGE,0,0,0,0);
            
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
    init_theta3=0;
    [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
 theta_points=All_theta(:,2); 

All_theta=All_theta*pi/180;
what=All_theta(:,2);
% hh=directKinematics(what);
% hhh=ForwardKinematics(what,1);
robot.ApplyPosi2(what);

q = robot.GetState();
while(norm(directKinematics(q) - way_points(:,1)) > 0.1 )
    now_2=directKinematics(q);
    q = robot.GetState();
end
now_3=directKinematics(q)
qd = robot.GetState();

T_tot=(size(way_points,2)-1)*1;
tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(way_points,2)-1):T_tot;
% tpts = 0:T_tot/(2-1):T_tot;

% % 续上速度
desired_v32=[zeros(3,size(way_points,2))];
desired_v72=[zeros(7,size(way_points,2))];
[points,points_dot,points_dotdot,pp] = cubicpolytraj(way_points,tpts,tvec,...
                'VelocityBoundaryCondition', desired_v32);   
            
way_points;
[hang,liehh]=size(points);    

for i_inv = 1 : liehh
    init_theta1=180;
    init_theta2=0;
    init_theta3=0;
    xd=points(1,i_inv);
    yd=points(2,i_inv);
    zd=points(3,i_inv);
    [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
%      [All_theta] = inverse_with_gesture_zzz(xd,yd,zd,init_theta1,init_theta2,init_theta3).';
if size(All_theta,2) ~=0
    which_need=abs(All_theta(7,:)) >= 140;
    need_plus=All_theta(7,which_need);
    if need_plus < -140
        All_theta(7,which_need)=need_plus+180;
    elseif need_plus > 140
        All_theta(7,which_need)=need_plus-180;
    end
end 
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
theta_points=theta_points.*pi/180;
theta_points(:,1)=[];
theta=theta_points;
FULL_theta=theta; % in 3.14
theta_dot=[desired_v72(:,1) (theta(:,3:end)-theta(:,1:end-2))/Ts/2 desired_v72(:,2)];
%%
%Parameters
T = 5;
nSamples = 100;
kPaths = 4;
convThr = 0;

%%
%Setup environment
jb=lynxStart();hold on;
%Environment size
obsts=[];
% %Passage hole [center r]
% hole = [0 0 200 60];
hole=[];
%Calculate EDT_Env
voxel_size = [10, 10, 10];
 [Env,all_cubes] = constructEnv(voxel_size, myspace, cartis_obs);           
 
Env_edt = prod(voxel_size) ^ (1/3) * sEDT_3d(Env);
% Env_edt = sEDT_3d(Env);

%%
%Initialization
start_joint=FULL_theta(:,1);  % 这个FULL的结果似乎有抖动啊
end_joint=FULL_theta(:,6*20);

% 老版本的初始轨迹
qStart=start_joint';
% qStart=qStart*180/pi;
qGoal=end_joint';
% qGoal=qGoal*180/pi;
theta = [linspace(qStart(1), qGoal(1), nSamples);linspace(qStart(2), qGoal(2), nSamples);linspace(qStart(3), qGoal(3), nSamples);...
    linspace(qStart(4), qGoal(4), nSamples);linspace(qStart(5), qGoal(5), nSamples);linspace(qStart(6), qGoal(6), nSamples);linspace(qStart(7), qGoal(7), nSamples)];
%老杨试试mimjerk
% desired_v72=[zeros(7,2)];
% [points,points_dot,points_dotdot,pp] = cubicpolytraj(processed_way_points,tpts,tvec,...
%                 'VelocityBoundaryCondition', zeros(3,2));   


%Initialize theta on a line
ntheta = cell(kPaths, 1);

%%
%Precompute
A_k = eye(nSamples - 1, nSamples - 1);
A = -2 * eye(nSamples, nSamples);
A(1:nSamples - 1, 2:nSamples) = A(1:nSamples - 1, 2:nSamples) + A_k;
A(2:nSamples, 1:nSamples - 1) = A(2:nSamples, 1:nSamples - 1) + A_k;
A = A(:, 2:99);
R = A' * A;
Rinv = inv(R);
M = 1 / nSamples * Rinv ./ max(Rinv, [], 1);
Rinv = Rinv / sum(sum(Rinv));

%%
%Planner
Q_time = [];
RAR_time = [];
Qtheta = stompCompute_PathCost(theta, obsts, hole, R, Env_edt);
QthetaOld = 0;
tic
ite=0;
while 1

    ite=ite+1;
%     Qtheta
    QthetaOld = Qtheta;

    %Random Sampling
    [ntheta, epsilon] = stompCompute_NoisyTraj(kPaths,qStart,qGoal,Rinv, theta);

    %Compute Cost and Probability
    pathCost = zeros(kPaths, nSamples);
    pathE = zeros(kPaths, nSamples);
    pathProb = zeros(kPaths, nSamples);
    for i = 1 : kPaths
        % ???
        pathCost(i, :) = stompCompute_Cost(ntheta{i}, obsts, hole, Env_edt);
    end
    pathE = stompCompute_ELambda(pathCost);
    pathProb = pathE ./ sum(pathE, 1);
    pathProb(isnan(pathProb) == 1) = 0;
    
    
    %Compute delta
    dtheta = sum(pathProb .* epsilon, 1);
    
    if sum(sum(pathCost)) == 0
        dtheta = zeros(nSamples);
    end
    
    %Smooth delta
    dtheta = M * dtheta(2 : nSamples - 1)';
    
    %Update theta
    theta(:, 2 : nSamples - 1) = theta(:, 2 : nSamples - 1) + [dtheta';dtheta';dtheta';dtheta';dtheta';dtheta';dtheta'];
%     theta
    
    %Compute new trajectory cost ???
    Qtheta = stompCompute_PathCost(theta, obsts, hole, R, Env_edt);
    

    Qtheta;
    Q_time = [Q_time Qtheta];
    RAR = 1/2 * sum(sum(theta(:, 2:99) * R * theta(:, 2:99)'));
    RAR_time = [RAR_time RAR];
    if ite > 20 || sum(sum(dtheta)) == 0
        break
    end

end
disp('We finished!!!!!!!!!!!!!!!');
toc
%%
% Visualization

% plotObstacle([140 140;180 180;280 280],35,1);
% plotObstacle([220 220;100 100;200 200],35,1);
disp(['iteration:',num2str(ite)]);

for num_cube=1:size(all_cubes,1)/2
    Cube=all_cubes(2*num_cube-1:2*num_cube,:);
% fill3([100 100 1000 1000],[-1000 1000 1000 -1000],[], 'r')
 fill3([Cube(1,1) Cube(1,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1)], [Cube(1,2) Cube(1,2)+Cube(2,2)... 
     Cube(1,2)+Cube(2,2) Cube(1,2) ], [Cube(1,3) Cube(1,3) Cube(1,3) Cube(1,3)], 'b'); hold on;
  fill3([Cube(1,1) Cube(1,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1)], [Cube(1,2) Cube(1,2)+Cube(2,2)... 
     Cube(1,2)+Cube(2,2) Cube(1,2) ], [Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3)], 'b'); hold on;
end
 
 
 
 
for i= 1: length(theta)
    [X,~]=updateQ(theta(:,i)');
    plot3(X(1, 1), X(1, 2), X(1, 3), 'bo', 'markersize', 6);
    plot3(X(2, 1), X(2, 2), X(2, 3), 'ro', 'markersize', 6);
    plot3(X(3, 1), X(3, 2), X(3, 3), 'go', 'markersize', 6);
    plot3(X(4, 1), X(4, 2), X(4, 3), 'yo', 'markersize', 6);
    plot3(X(5, 1), X(5, 2), X(5, 3), 'ko', 'markersize', 6);
    plot3(X(6, 1), X(6, 2), X(6, 3), 'mo', 'markersize', 6);
    plot3(X(7, 1), X(7, 2), X(7, 3), 'bo', 'markersize', 6);
    plot3(X(8, 1), X(8, 2), X(8, 3), 'bo', 'markersize', 6);
    lynxServoSim(theta(1,i),theta(2,i),theta(3,i),theta(4,i),theta(5,i),theta(6,i),theta(7,i));
% lynxServoSim(theta(:,i)');
pause(0.01);
end

figure;
plot(1:ite, Q_time, 'b', 'linewidth', 2);hold on;
figure;
plot(1:ite, RAR_time, 'r', 'linewidth', 2);

% THETA_DOT=[(theta(:,3:end)-theta(:,1:end-2))/2/Ts];
THETA_DOT=[(theta(:,2:end)-theta(:,1:end-1))/Ts zeros(7,1)];
t=0;lastCmdTime=0;
for i=1:size(THETA_DOT,2)
    pause(Ts);
    this_point=ceil(t/T_tot*size(THETA_DOT,2));
    
            currentCmdTime = robot.GetLastCmdTime();
        dt = (currentCmdTime-lastCmdTime)/1000;
        % get states feedback
    feedback_joint_position=robot.GetState();
    feedback_joint_velocity=robot.GetV();
    robot.ApplyControl(THETA_DOT(:,this_point+1), Ts); % velocity control
    
        lastCmdTime = currentCmdTime;
        t = t+dt;
end
 