
close all;clear;clc;
addpath('C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client_rawing')
addpath('C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client_rawing\EMG_IMU')
gprMdl_x1_intent2=load('gprMdl20220127_2_1to1.mat').gprMdl_x1;
gprMdl_x2_intent2=load('gprMdl20220127_2_1to1.mat').gprMdl_x2;
gprMdl_x1_intent1=load('gprMdl20220127_1_1to1.mat').gprMdl_x1;
gprMdl_x2_intent1=load('gprMdl20220127_1_1to1.mat').gprMdl_x2;
%%
IP_remote_IMU = "10.0.8.150"; 
port_remote_IMU = 30000;
IP_local_IMU = "172.31.1.100"; 
port_local_IMU = 30000;
Role_IMU = 'client';
t_client_EMG_local = tcpip(IP_local_IMU,port_remote_IMU,...
                'NetworkRole',Role_IMU,...
                'TimeOut',200);
t_client_EMG_local.InputBuffersize=100000;
t_client_EMG_local.OutputBuffersize=100000;
disp(['未打开！',datestr(now)])
fopen(t_client_EMG_local);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])



direction=1;

MY_COUNT=0; judge2=5000; judge3=15000;
while 1
    pause(0.02);
    MY_COUNT=MY_COUNT+1;
    y_new=MY_COUNT;

    
    if  t_client_EMG_local.BytesAvailable>0
        data_recv_IMU = fread(t_client_EMG_local,t_client_EMG_local.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
        
        which_head_IMU=find(88887<=data_recv_IMU);
        if ~isempty(which_head_IMU)
            which_head2_IMU=which_head_IMU(end);
            this_frame_IMU=data_recv_IMU(which_head2_IMU:end);     
            start=this_frame_IMU(1);
            if start ==7654321
                break
            end
            intent=this_frame_IMU(2);
            in_step=this_frame_IMU(3);
            if in_step == 14
                if intent == 1
                    result = myGRP_onlyx_limitfuture(start, intent, gprMdl_x1_intent2);
                else
                    result = myGRP_onlyx_limitfuture(start, intent, gprMdl_x2_intent2);
                end
            elseif in_step == 25
                if intent == 1
                    result = myGRP_onlyx_limitfuture(start, intent, gprMdl_x1_intent1);
                else
                    result = myGRP_onlyx_limitfuture(start, intent, gprMdl_x2_intent1);
                end
            end
            fwrite(t_client_EMG_local,[result.'],'double');%写入数字数据，每次发送360个double
        end        
    end

end


disp('Transport is done!');
fclose(t_client_EMG_local); 



