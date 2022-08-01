%%
%工作空间
clear;
clc;
% %建立机器人模型
% % theta d a alpha offset
% L1=Link([0 0 0 0 0 ],'modified'); %连杆的D-H参数
% L2=Link([0 0.14909 0 -pi/2 0 ],'modified');
% L3=Link([0 0 0.4318 0 0 ],'modified');
% L4=Link([0 0.43307 0.02032 -pi/2 0 ],'modified');
% L5=Link([0 0 0 pi/2 0 ],'modified');
% L6=Link([0 0 0 -pi/2 0 ],'modified');
% robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','puma560','base' , ...
% transl(0, 0, 0.62)* trotz(0)); %连接连杆，机器人取名puma560
% 
% % 参数
%     %关节角限位
%     q1_s=-160; q1_end=160;
%     q2_s=-225; q2_end=45;
%     q3_s=-45;  q3_end=225;
%     q4_s=-110; q4_end=170;
%     q5_s=-100;  q5_end=100;
%     q6_s=-266;  q6_end=266;
%     
%     %计算点数
%     num=50000;
%  
% % 求取工作空间
%     %设置轴关节随机分布,轴6不对工作范围产生影响，设置为0
%     q1_rand = q1_s + rand(num,1)*(q1_end - q1_s);
%     q2_rand = q2_s + rand(num,1)*(q2_end - q2_s);
%     q3_rand = q3_s + rand(num,1)*(q3_end - q3_s);
%     q4_rand = q4_s + rand(num,1)*(q4_end - q4_s);
%     q5_rand = q5_s + rand(num,1)*(q5_end - q5_s);
%     q6_rand = q6_s + rand(num,1)*(q6_end - q6_s);
%     q = [q1_rand q2_rand q3_rand q4_rand q5_rand q6_rand];
%     
%     %正运动学计算工作空间
%     tic;
%     T_cell = cell(num,1);
%     [T_cell{:,1}]=robot.fkine(q).t;%正向运动学仿真函数
%     disp(['运行时间：',num2str(toc)]);
%  
%  % 分析结果
%     %绘制工作空间
%     t1=clock;
%     figure('name','机械臂工作空间')
%     hold on
%     plotopt = {'noraise', 'nowrist', 'nojaxes', 'delay',0};
%     robot.plot([0 0 0 0 0 0], plotopt{:});
%      figure_x=zeros(num,1);
%      figure_y=zeros(num,1);
%      figure_z=zeros(num,1);
%      for cout=1:1:num
%          figure_x(cout,1)=T_cell{cout}(1);
%          figure_y(cout,1)=T_cell{cout}(2);
%          figure_z(cout,1)=T_cell{cout}(3);
%      end
%      plot3(figure_x,figure_y,figure_z,'r.','MarkerSize',3);
%      hold off
%      disp(['绘制工作空间运行时间：',num2str(etime(clock,t1))]);  
%      
%      %获取X,Y,Z空间坐标范围
%      Point_range=[min(figure_x) max(figure_x) min(figure_y) max(figure_y) min(figure_z) max(figure_z)];
%      disp(['X在空间坐标范围:',num2str(Point_range(1:2))]);
%      disp(['Y在空间坐标范围:',num2str(Point_range(3:4))]);
%      disp(['Z在空间坐标范围:',num2str(Point_range(5:6))]);


% Create two platforms
platform1 = collisionBox(0.4,0.4,0.4);
platform1.Pose = trvec2tform([0.7 0 0.2]);


% Store in a cell array for collision-checking
worldCollisionArray = {platform1};

% ax = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);
robot = loadrobot("kukaIiwa14","DataFormat","column","Gravity",[0 0 -9.81]);
% show(robot,homeConfiguration(robot),"Parent",ax);

% startPose = trvec2tform([0, 0, 1.4100])*axang2tform([1 0 0 pi]);
% endPose = trvec2tform([0.65,0,0.435])*axang2tform([1 0 0 pi]);
% Use a fixed random seed to ensure repeatable results
rng(0);
ik = inverseKinematics("RigidBodyTree",robot);
weights = ones(1,6);
% startConfig = ik("iiwa_link_ee_kuka",startPose,weights,robot.homeConfiguration);

%%
point_2=[-0.4013;0.8987;0;-0.3310;0;1.9106;-0.3927];
point_above=[   -0.1748;0.6793;0;-1.0330;0;1.4280;-0.3927];
point_collision=[   -0.1748;0.7447;0;-1.1795;0;1.2161;-0.3927];
% startConfig=[0;0;0;0;0;0;0]
% endConfig = ik("iiwa_link_ee_kuka",endPose,weights,robot.homeConfiguration);
endConfig=point_collision;

%Show initial and final positions
% show(robot,startConfig);
% show(robot,endConfig);
% q = trapveltraj([homeConfiguration(robot),startConfig,endConfig],200,"EndTime",2);
q=point_collision;
% Initialize outputs
inCollision = false(length(q), 1); % Check whether each pose is in collision
worldCollisionPairIdx = cell(length(q),1); % Provide the bodies that are in collision





% for i = 1:length(q)
%     
i=1;
    [inCollision(i),sepDist] = checkCollision(robot,q(:,i),worldCollisionArray,"IgnoreSelfCollision","on","Exhaustive","on");
    
    [bodyIdx,worldCollisionObjIdx] = find(isnan(sepDist)); % Find collision pairs
    worldCollidingPairs = [bodyIdx,worldCollisionObjIdx]; 
    worldCollisionPairIdx{i} = worldCollidingPairs;
    
% end
isTrajectoryInCollision = any(inCollision)

% collidingIdx1 = find(inCollision,1);
% collidingIdx2 = find(inCollision,1,"last");
% 
% % Identify the colliding rigid bodies.
% collidingBodies1 = worldCollisionPairIdx{collidingIdx1}*[1 0]';
% collidingBodies2 = worldCollisionPairIdx{collidingIdx2}*[1 0]';
% 
% % Visualize the environment.
% ax = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);
% 
% % Add the robotconfigurations & highlight the colliding bodies.
% show(robot,q(:,collidingIdx1),"Parent",ax,"PreservePlot",false);
% exampleHelperHighlightCollisionBodies(robot,collidingBodies1 + 1,ax);
% show(robot,q(:,collidingIdx2),"Parent"',ax);
% exampleHelperHighlightCollisionBodies(robot,collidingBodies2 + 1,ax);