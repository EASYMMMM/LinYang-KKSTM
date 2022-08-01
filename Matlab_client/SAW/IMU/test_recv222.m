close all;clear;clc;

t_server_IMU1=tcpip('0.0.0.0',40000,'NetworkRole','server');%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
t_server_IMU1.InputBuffersize=100000;
disp(['未打开！',datestr(now)])
fopen(t_server_IMU1);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])

IP_remote_IMU = "10.0.8.150"; 
port_remote_IMU = 30000;
IP_local_IMU = "172.31.1.100"; 
port_local_IMU = 30000;
Role_IMU = 'server';
t_server_IMU2 = tcpip(IP_local_IMU,port_remote_IMU,...
                'NetworkRole',Role_IMU,...
                'TimeOut',20);

t_server_IMU2.InputBuffersize=100000;
t_server_IMU2.OutputBuffersize=100000;
disp(['未打开！',datestr(now)])
fopen(t_server_IMU2);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])



jjj=0;
data_all_IMU=[];count_right_IMU=0;
%%
% while jjj<800   
%     
%     jjj=jjj+1;
%     pause(0.02);
%     tic
%     if  t_server_IMU1.BytesAvailable>0
%         data_recv_IMU = fread(t_server_IMU1,t_server_IMU1.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
%         
%         which_head_IMU=find(88887<=data_recv_IMU);
%         if ~isempty(which_head_IMU)
%             which_head2_IMU=which_head_IMU(end);
%             this_frame_IMU=data_recv_IMU(which_head2_IMU:end);
%             if length(this_frame_IMU) == 3
%                 
%                 count_right_IMU = count_right_IMU + 1;
%                 data_all_IMU(:,count_right_IMU) = this_frame_IMU;
%             end
%         end
%     end
%     
%     %%
%     if  t_server_IMU2.BytesAvailable>0
%         data_recv_IMU2 = fread(t_server_IMU2,t_server_IMU2.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
%         
%         which_head_IMU2=find(88887<=data_recv_IMU2);
%         if ~isempty(which_head_IMU2)
%             which_head2_IMU2=which_head_IMU2(end);
%             this_frame_IMU2=data_recv_IMU2(which_head2_IMU2:end);
%             this_frame_IMU2;
%         end
%     end    
%     toc
% end
%%
intent_l=1;
while jjj<1800   
    jjj=jjj+1;
    pause(0.02);
    
    if  t_server_IMU1.BytesAvailable>0
        data_recv_IMU = fread(t_server_IMU1,t_server_IMU1.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
        
        which_head_IMU=find(88887<=data_recv_IMU);
        if ~isempty(which_head_IMU)
            which_head2_IMU=which_head_IMU(end);
            this_frame_IMU1=data_recv_IMU(which_head2_IMU:end)
        end    
    end
    
    
    if  t_server_IMU2.BytesAvailable>0
        data_recv_IMU = fread(t_server_IMU2,t_server_IMU2.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
        data_recv_IMU

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
        fwrite(t_server_IMU2,[88888.888,start,intent],'double');%第一个数据头，然后当前点，然后意图
    end
    
    
end


fclose(t_server_IMU1);
fclose(t_server_IMU2);