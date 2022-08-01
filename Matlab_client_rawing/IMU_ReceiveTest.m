%% IMU Receive test
% 测试IMU数据接收是否正常

%% Connect with IMU
disp('Wait for IMU');

t_server_IMU = IMU_Connect(); % 连接IMU数据收集副电脑 
disp('IMU Connected!');
i  = 1;
this_frame_IMU = 0;
while 1
    
%     [forearm_imu_data , upperarm_imu_data , mass_imu_data , imu_flag ] = IMU_ReadOneFrame(t_server_IMU); 
%     forearm_imu_data

if  t_server_IMU.BytesAvailable>0
    data_recv_IMU = fread(t_server_IMU, t_server_IMU.BytesAvailable/8 ,'double');% 第二个参数代表 要从缓冲区里读取多少个 double，一个double为8个字节

    head=find(88887<=data_recv_IMU);    %寻找帧头。帧头为88888.888
    if ~isempty(head)
        frame_head = head(end);                                                  %定位到最新一帧
        this_frame_IMU=data_recv_IMU(frame_head:end);  
    end
end
    this_frame_IMU
    i = i+1;
    pause(1)
    if i>20
        break;
    end
    
end

fclose(t_server_IMU);