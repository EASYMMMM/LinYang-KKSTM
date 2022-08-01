close all;clear;clc;
% t_server_IMU1=tcpip('169.254.205.200',30000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
% t_server_IMU2=tcpip('0.0.0.0',40000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
% t_server_IMU1.InputBuffersize=100000;
% disp(['未打开！',datestr(now)])
% fopen(t_server_IMU1);%打开服务器，直到建立一个TCP连接才返回；
% disp(['已打开！',datestr(now)])




t_server_GPR = tcpip("172.31.1.100",30000,...
                'NetworkRole','server',...
                'TimeOut',20);

t_server_GPR.InputBuffersize=100000;
t_server_GPR.OutputBuffersize=100000;
disp(['未打开GPR！',datestr(now)])
fopen(t_server_GPR);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开GPR！',datestr(now)])


jjj=0;intent_l=1;
data_all_IMU=[];count_right_IMU=0;
%%
while jjj<1800   
    jjj=jjj+1;
    pause(0.02);
    if  t_server_GPR.BytesAvailable>0
        data_recv_IMU = fread(t_server_GPR,t_server_GPR.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
        data_recv_IMU
%         which_head_IMU=find(88887<=data_recv_IMU);
%         if ~isempty(which_head_IMU)
%             which_head2_IMU=which_head_IMU(end);
%             this_frame_IMU=data_recv_IMU(which_head2_IMU:end);
%             this_frame_IMU
%         end
    end
    
    
    if rem(jjj,600) == 0
        
        intent_l=-intent_l
        if intent_l > 0
            start = 0.16;
            intent=2;
        else
            start = 0.03;
            intent=1;
        end
        fwrite(t_server_GPR,[88888.888,start,intent],'double');%第一个数据头，然后当前点，然后意图
    end
    
    
end

fclose(t_server_GPR);
