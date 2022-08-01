close all;clear;clc;
% t_server_IMU1=tcpip('169.254.205.200',30000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
% t_server_IMU2=tcpip('0.0.0.0',40000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
% t_server_IMU1.InputBuffersize=100000;
% disp(['未打开！',datestr(now)])
% fopen(t_server_IMU1);%打开服务器，直到建立一个TCP连接才返回；
% disp(['已打开！',datestr(now)])


IP_remote_IMU = "10.0.8.150"; 
port_remote_IMU = 30000;
IP_local_IMU = "172.31.1.100"; 
port_local_IMU = 30000;
Role_IMU = 'server';
t_server_IMU2 = tcpip(IP_local_IMU,port_remote_IMU,...
                'NetworkRole',Role_IMU,...
                'TimeOut',20);

t_server_IMU2.InputBuffersize=100000;

disp(['未打开！',datestr(now)])
fopen(t_server_IMU2);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])


jjj=0;
data_all_IMU=[];count_right_IMU=0;
%%
while jjj<800   
    jjj=jjj+1;
    pause(0.02);
    if  t_server_IMU2.BytesAvailable>0
        data_recv_IMU = fread(t_server_IMU2,t_server_IMU2.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
        
        which_head_IMU=find(88887<=data_recv_IMU);
        if ~isempty(which_head_IMU)
            which_head2_IMU=which_head_IMU(end);
            this_frame_IMU=data_recv_IMU(which_head2_IMU:end);
            if length(this_frame_IMU) == 3
                
                count_right_IMU = count_right_IMU + 1
                data_all_IMU(:,count_right_IMU) = this_frame_IMU;
            end
        end
    end
    
    
    
    fwrite(t_server_IMU2,[88888.888,555,555],'double');%写入数字数据，每次发送360个double

end

fclose(t_server_IMU2);
