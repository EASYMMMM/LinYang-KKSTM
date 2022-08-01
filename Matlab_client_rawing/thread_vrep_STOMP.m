% 这个是副线程，每当主线程传过来当前位置，这个STOMP就会计算轨迹，再发回去，还没加GPR

clear;
clc;close all;
%% 传数主机IP
t_server_self=tcpip('0.0.0.0',30000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
t_server_self.InputBuffersize=51200;

t_server_self.OutputBufferSize=51200;
disp(['未打开！',datestr(now)])
fopen(t_server_self);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])

refer_pose=[0;48.2999158357778;0;-80.4847794064853;0;-42.7846952422630;0]*pi/180;
FUTURE=3;
addpath('C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client_rawing\EMG_IMU')
addpath 'C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main'
addpath 'C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client'
addpath 'C:\Lin YANG\from me\KUKA\KUKA_Matlab\TRO\STOMP_done\Intro-to-Robo-Proj-master'
CHANGE=0;
now_pos_3=[0.1 -0.45 0.2]';

[lastq,lastR,total_act way_points which_state_now myspace cartis_obs OBSTACLE]= RL2m3m3_maze(now_pos_3,0,CHANGE,0,0,0,0,[]);
        
%% STOMP
%Parameters
T = 0.5;
nSamples = 50;
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

last_space=myspace;
last_q=lastq;
last_R=lastR;
OVERR=0;
LOOP=0;
while OVERR == 0
    pause(0.002);
    LOOP=LOOP+1;

        
    
    if  t_server_self.BytesAvailable>0
        disp('I receive the request!')
        data_recv_self = fread(t_server_self,t_server_self.BytesAvailable/8,'double');%    disp(size(data_recv));
        q=data_recv_self(8:end);
        desired_q=data_recv_self(1:7);
        [ poseow, nsparam, rconf, jout ] = ForwardKinematics( q, 1 );
        start_state = poseow(1:3,4);
%         start_state=data_recv_self;  % please send the current space 3*1 to this thread
        %         start_state=[0.1; -0.45; 0.2];
        CHANGE=1;
        [lastq,lastR,total_act way_points which_state_now myspace cartis_obs OBSTACLE]= RL2m3m3_maze(start_state,last_space,CHANGE,last_q,last_R,0,0,OBSTACLE);
           
        theta_points=refer_pose*180/pi;
        if size(way_points,2) > FUTURE
            points=[way_points(:,1) way_points(:,FUTURE)];
        else
            OVERR = 1;
            points=[way_points(:,1) way_points(:,end)];
        end
        
        for i_inv = 1 : 2
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
        FULL_theta=theta_points;
        
        
        
        
        
        %%
        %Initialization
        start_joint=desired_q;  % 这个FULL的结果似乎有抖动啊
        end_joint=FULL_theta(:,2);
        
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
        A = A(:, 2:nSamples-1);
        R = A' * A;
        Rinv = inv(R);
        M = 1 / nSamples * Rinv ./ max(Rinv, [], 1);
        Rinv = Rinv / sum(sum(Rinv));
        
        %%
        %Planner
        Q_time = [];
        RAR_time = [];
        Qtheta = stompCompute_PathCost(theta, obsts, hole, R, Env_edt,nSamples);
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
            Qtheta = stompCompute_PathCost(theta, obsts, hole, R, Env_edt,nSamples);
            
            
            Qtheta;
            Q_time = [Q_time Qtheta];
            RAR = 1/2 * sum(sum(theta(:, 2:nSamples-1) * R * theta(:, 2:nSamples-1)'));
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
        
        [column1 row1]=size(theta);
        result=reshape(theta,1,column1*row1);
        fwrite(t_server_self,[result],'double');%写入数字数据，每次发送360个double
        
    end
end
disp('Transport is done!');
fwrite(t_server_self,[87654321],'double');%写入数字数据，每次发送360个double        
fclose(t_server_self);












