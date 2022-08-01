%% 
IP_remote = "192.168.11.1"; 
port_remote = 5000;
IP_local = "192.168.11.2"; 
port_local = 5000;
Role = 'client';
t_server = tcpip(IP_remote,port_remote,...
                'NetworkRole',Role,...
                'LocalPort',port_local,...
                'TimeOut',20,...
                'InputBufferSize',8192);

t_server.InputBuffersize=100000;

disp(['未打开！',datestr(now)])
fopen(t_server);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])
%get(t_server);
data_all = [];count_right = 0;


data_all_IMU=[];count_right_IMU=0;
%%
while 1      
    if  t_server.BytesAvailable>0
        data_recv_IMU = fread(t_server_IMU,t_server_IMU.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
        count_right_IMU = count_right_IMU + 1;
        which_head_IMU=find(88887<=data_recv_IMU);
        if ~isempty(which_head_IMU)
            which_head2_IMU=which_head_IMU(end);
            this_frame_IMU=data_recv_IMU(which_head2_IMU:end);
            if length(this_frame_IMU) == 76
                data_all_IMU(:,count_right_IMU) = this_frame_IMU;
            end
        end
    end
end

fclose(t_server);

    