close all;clear;clc;

IP_remote_IMU = "192.168.11.1"; 
port_remote_IMU = 5000;
IP_local_IMU = "192.168.11.2"; 
port_local_IMU = 5000;
Role_IMU = 'client';
t_server_IMU = tcpip(IP_remote_IMU,port_remote_IMU,...
                'NetworkRole',Role_IMU,...
                'LocalPort',port_local_IMU,...
                'TimeOut',20,...
                'InputBufferSize',8192);

t_server_IMU.InputBuffersize=100000;
addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main-raw')  
addpath('C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client_rawing\EMG_IMU')

disp(['未打开！',datestr(now)])
fopen(t_server_IMU);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])

STOPP=0;
data_all_IMU=[];count_right_IMU=0;
ori_angles=[];
%%
all_y12=[];all_a12=[];all_b12=[];
all_y23=[];all_a23=[];all_b23=[];
all_y13=[];all_a13=[];all_b13=[];
while STOPP < 3000    
    pause(0.005);
    STOPP=STOPP+1;
            if  t_server_IMU.BytesAvailable>0
                STOPP=0;
                data_recv_IMU = fread(t_server_IMU,t_server_IMU.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
                
                which_head_IMU=find(88887<=data_recv_IMU);
                if ~isempty(which_head_IMU)
                    which_head2_IMU=which_head_IMU(end);
                    this_frame_IMU=data_recv_IMU(which_head2_IMU:end);
                    
                    %% 
                    where_obstacles=find(this_frame_IMU <= -12344);
                    if length(where_obstacles)>0
                        obstacles=this_frame_IMU(where_obstacles+1:end)
                    end
                    
                    if length(this_frame_IMU) == 76
                        
                        count_right_IMU = count_right_IMU + 1
                        data_all_IMU(:,count_right_IMU) = this_frame_IMU;
                        ori_angles(:,count_right_IMU) = this_frame_IMU([14:16 39:41 64:66]);
                        
                        
                        
                        this1=this_frame_IMU(1+1:25+1,:).';
                    this2=this_frame_IMU(26+1:50+1,:).';
                    this3=this_frame_IMU(51+1:75+1,:).';              
                        R1 = rotate_matrix(this1);
                        R2 = rotate_matrix(this2);
                        R3 = rotate_matrix(this3);
                        R13 = R3.'*R1;
                        R23 = R3.'*R2;
                        R12 = R2.'*R1;

                        
                        [b12,a12,y12] = inverse_angle(R12);
                        [b23,a23,y23] = inverse_angle(R23);
                        [b13,a13,y13] = inverse_angle(R13);
                        all_a12=[all_a12 a12];
                        all_b12=[all_b12 b12];
                        all_y12=[all_y12 y12];
                        all_a23=[all_a23 a23];
                        all_b23=[all_b23 b23];
                        all_y23=[all_y23 y23];
                        all_a13=[all_a13 a13];
                        all_b13=[all_b13 b13];
                        all_y13=[all_y13 y13];
                        
                        
                    end
                end
      
            end
    
    
end
fwrite(t_server_IMU,[88888.888,7654132],'double');%写入数字数据，每次发送360个double
fclose(t_server_IMU);
ori_angles2=ori_angles.';




figure;
plot(all_y23); hold on;
plot(all_b23); hold on;
plot(all_a23)
legend('y23','b23','a23')


figure;
plot(all_y12); hold on;
plot(all_b12); hold on;
plot(all_a12)
legend('y12','b12','a12')

figure;
plot(all_y13); hold on;
plot(all_b13); hold on;
plot(all_a13)
legend('y13','b13','a13')