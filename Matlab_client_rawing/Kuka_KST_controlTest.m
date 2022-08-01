%% Kuka_KST_controlTest
% 尝试用KST类控制KUKA
% 7.1


close all;
clear;
clc;
warning('off')

%% Connect with IMU
t_server_IMU = IMU_Connect(); % 连接IMU数据收集副电脑 
disp('IMU Connected!');


folderpath='E:\MMMLY\IMU\测试\单手臂测试\'; %标定数据存放位置
expdate='20220622'; %标定数据文件夹名称
datapath=[folderpath,expdate,'\'];
[R_U,R_F]=OrientArm(datapath,0); % 计算旋转矩阵，详见help OrientArm
disp('标定完成！')
pause(2);
%参数设定
Pmass        = [-0.15;0;0];  %重物的方向向量 基于重物自身坐标系
Pforearm    = [ 0 ; 0.27 ; 0]; %小臂的方向向量 基于小臂自身坐标
Pupperarm = [ 0 ; 0.30 ; 0]; %大臂的方向向量 基于大臂自身坐标系
%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange.
% Tef_flange(3,4)=30/1000;
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object


%% Start a connection with the server
flag=iiwa.net_establishConnection();
if flag==0
  return;
end
disp('Kuka connected!');
pause(1);



%% Go to initial configuration

relVel=0.25; % over ride relative joint velocities

%pos={0, -pi / 180 * 10, 0, -pi / 180 * 100, pi / 180 * 90,pi / 180 * 90, 0};   % initial cofiguration

pos={0., pi / 180 * 30, 0, -pi / 180 * 60, 0,pi / 180 * 90, 0};
iiwa.movePTPJointSpace( pos, relVel); % go to initial position

disp('初始');
init_eefCartPos = iiwa.getEEFCartesianPosition();     %初始末端坐标


[eef_T, eef_jacobian ] = iiwa.gen_DirectKinematics(cell2mat(pos));  %正运动学求解
init_eef_cart = eef_T(1:3,4)   %末端执行器笛卡尔坐标






%% Start direct servo in joint space       
iiwa.realTime_startVelControlJoints();

%% Parameter setting
 
runTime = 16;
timeInt   = 0.02;
timeVec  = [0:0.02:runTime];
waypoint = [ ]; %路径点
waypoint(:,1) = init_eef_cart ;
waypoint(:,2) = waypoint(:,1) + [0 ; 0.15 ; 0  ];
waypoint(:,3) = waypoint(:,2) + [0 ;  0  ;-0.15];
waypoint(:,4) = waypoint(:,3) + [0 ;-0.15 ; 0  ];
waypoint(:,5) = waypoint(:,4) + [0 ;  0  ; 0.15];
timepoint = linspace(0,runTime,5);

[eefTarget ,eefTargetd ,eefTargetdd, pp] = cubicpolytraj(waypoint,timepoint ,timeVec);

%% Control Loop

disp('开始');
a=datevec(now);
timeOrigin=a(6)+a(5)*60+a(4)*60*60; %初始时间戳
totalLoop = length(timeVec);
i = 1;
figure_i = 1;

while (1)
    
    a=datevec(now);
    timeNow=a(6)+a(5)*60+a(4)*60*60 - timeOrigin;   %当前时间戳
    if ~(timeNow > timeVec(i))
        continue
    end
    jPos = iiwa.getJointsPos();                              %读取角度
    [eefT, eefJacobian ] = iiwa.gen_DirectKinematics(cell2mat(jPos));
    JVel = eefJacobian(1:3,:);        %速度雅各比
    jPosd = pinv(JVel) * eefTargetd(:,i);
    iiwa.sendJointsVelocities(num2cell(jPosd));  %输出关节速度
    i = i+1;
    
%         % 读取IMU数据
%     [forearm_imu_data , upperarm_imu_data , mass_imu_data , imu_flag ] = IMU_ReadOneFrame(t_server_IMU); 
%     forearm_imu_data
%     if imu_flag
%         R_forearm     =  rotate_matrix(forearm_imu_data);
%         R_upperarm  =  rotate_matrix(upperarm_imu_data);
%         R_mass         =  rotate_matrix(mass_imu_data);
%     end
    
      %%%%%%%%%%%%%%%%%%-----------画图--------------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%         joint_p =  num2cell(jPosd);                 %获取各个关节位置
%         qs=zeros(7,1);
%         for i=1:7
%             qs(i)=joint_p{i};
%         end
%         T = directKinematicsAboutEachJoint(qs);    %获取全部七个关节的变换矩阵
%          joint_cart = zeros(3,7);
% 
%         %重物  
%         Tmass = zeros(4,4);
%         Tmass(1:3,1:3) = R_mass;
%         Tmass(1:3,4) =  Tmass(1:3,1:3)  * Pmass + T(1:3,4,7); %重物方向沿末端执行器坐标系的x轴负方向
%         Tmass(4,4) = 1;
%         T(:,:,8) = Tmass;   
%         
%         %前臂
%         Tforearm = zeros(4,4);
%         Tforearm(1:3,1:3) = R_forearm;
%         Tforearm(1:3,4) = Tforearm(1:3,1:3) * Pforearm + T(1:3,4,8); %小臂的变换矩阵
%         Tforearm(4,4) = 1;
%         T(:,:,9) = Tforearm;
%         
%         %大臂
%         Tupperarm = zeros(4,4);
%         Tupperarm(1:3,1:3) = R_upperarm;
%         Tupperarm(1:3,4) = Tupperarm(1:3,1:3) * Pupperarm + T(1:3,4,9); %大臂的变换矩阵
%         Tupperarm(4,4) = 1;
%         T(:,:,10) = Tupperarm;
%         
%         % **￥@%@%！%@！%#@%！%@#%@#@#@#####￥#￥#@@￥@@@@@@@@2
%         for i = 1:10
%             joint_cart( : , i ) = T(1:3,4,i); 
%         end
%          joint_cart( : , 1 ) = [0;0;0];
%                
%         kuka_color = [240 153 80];    %为kuka选择喜欢的颜色
%         plot3(  joint_cart(1,1:7) ,  joint_cart(2,1:7) , joint_cart(3,1:7) ,'o-','color',kuka_color/255,'Linewidth',2);   %绘制KUKA机器人
%         hold on 
%         grid on
%         axis([-1.3 1.3 -1.3 1.3 -1.3 1.3]);
%         metal_color = [00 51 00];      %为金属重物选择喜欢的颜色
%         plot3(  joint_cart(1,7:8) ,  joint_cart(2,7:8) , joint_cart(3,7:8) ,'-','color',metal_color/255,'Linewidth',2);   %绘制重物
%         
%         arm_color = [255 106 106];   %为手臂选择喜欢的颜色 indian red
%         plot3( joint_cart(1,8:10) ,  joint_cart(2,8:10) , joint_cart(3,8:10) , 'o-','color', arm_color/255 , 'Linewidth',2); %绘制手臂
%         
%         end_point = [end_point , joint_cart( : , 7 ) ];  %机械臂末端轨迹线
%         endline_color = [102 153 255];  %为机械臂末端轨迹线选择喜欢的颜色
%         plot3(  end_point(1,:) ,  end_point(2,:) , end_point(3,:) ,'-','color',endline_color/255,'Linewidth',1);   %绘制机器人末端轨迹
%         
%         xlabel("X"); ylabel('Y');  zlabel('Z');
%         title("KUKA + 手臂 IMU绘图测试");
%         hold off
%         
%          frame=getframe(gcf);
%          imind=frame2im(frame);
%         [imind,cm] = rgb2ind(imind,256);
%           if figure_i==1
%               imwrite(imind,cm,['KUKA+Arm_IMUOnlineRebuild','.gif'],'gif', 'Loopcount',inf,'DelayTime',1e-6);
%           else
%               imwrite(imind,cm,['KUKA+Arm_IMUOnlineRebuild','.gif'],'gif','WriteMode','append','DelayTime',1e-6);
%           end              
%        
%         figure_i = figure_i+1;
        
        
    %%%%%%%%%%%%%%%%%%%%---------画图 END---------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if i > totalLoop
        break
    end
end


stopdq = [ 0; 0 ; 0;0;0;0;0];

ExTor = iiwa.sendJointsVelocitiesExTorques( num2cell(stopdq) );
disp('结束');

[eef_T, eef_jacobian ] = iiwa.gen_DirectKinematics(cell2mat(pos));  %正运动学求解
end_eef_cart = eef_T(1:3,4)   %末端执行器笛卡尔坐标
accuracy = end_eef_cart - init_eef_cart

%% turn off the server
iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer();
warning('on')
fclose(t_server_IMU);


