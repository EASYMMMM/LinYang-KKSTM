
close all;clear;clc;

addpath('C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\Matlab_client_rawing\EMG_IMU')

%%

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
flag_gprMdl_x25_int1=0;flag_gprMdl_x25_int2=0;
flag_gprMdl_x36_int1=0;flag_gprMdl_x36_int2=0;
flag_gprMdl_x14_int1=0;flag_gprMdl_x14_int2=0;
ROUNDD=0;last_start=0;last_in_step=0;
all_record=[]; all_xushang=[];all_result=[];

while 1
    pause(0.002);

    if  t_client_EMG_local.BytesAvailable>0
        tic
        ROUNDD=ROUNDD+1;
        result=0;
        data_recv_IMU = fread(t_client_EMG_local,t_client_EMG_local.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
        
        which_head_IMU=find(88887<=data_recv_IMU);
        if ~isempty(which_head_IMU)
            which_head2_IMU=which_head_IMU(end);
            this_frame_IMU=data_recv_IMU(which_head2_IMU:end);     
            nothing=this_frame_IMU(1);
            
            if nothing ==7654321
                break
            end
            record=this_frame_IMU(2:end-2);
            end_point=record(end);

            all_record=[all_record; record;];
            intent=this_frame_IMU(end-1);
            in_step=this_frame_IMU(end);

            
%% first training 
delta=((record(1))-(record(end)))/length(record);
extend_max=(record(1))+40*delta:-delta:(record(1));
extend_min=(record(end)):-delta:(record(end))-40*delta;
after_X111=[extend_max.'; record; extend_min(1:end-1).';];
after_Y111=[extend_max(2:end).'; record; record(end)*ones(41,1);];
train_L_X = [after_X111 intent*ones(size(after_X111,1),1)];
train_L_Y = after_Y111;

last_start=record(1);
            if in_step ~= last_in_step
                last_start=0;
            end

intent
% 1减少，2增加
in_step
            
            if in_step == 14
                
                if intent == 1
                    if flag_gprMdl_x14_int1 == 0
                        gprMdl_x14_int1 = fitrgp(train_L_X,train_L_Y);
                        flag_gprMdl_x14_int1=1;
                    else
                        all_train_L_14_int1_X=[last_train_14_X1; train_L_X;];
                        all_train_L_14_int1_Y=[last_train_14_Y1; train_L_Y;];
                        gprMdl_x14_int1= updateGPRMdl(gprMdl_x14_int1,all_train_L_14_int1_X, all_train_L_14_int1_Y);
                    end
                        last_train_14_X1=train_L_X;
                        last_train_14_Y1=train_L_Y;   
                        
                    if last_start ~= 0
                        Iuse_start1=last_start
                        result = myGRP_onlyx_limitfuture(last_start, 1, gprMdl_x14_int1,200);
                        my_result1=result(end)
                        result=[1; result;];
                    end
                else
                    if flag_gprMdl_x14_int2 == 0
                        init_train_L_X2=train_L_X;
                        init_train_L_Y2=train_L_Y;
                        gprMdl_x14_int2 = fitrgp(init_train_L_X2,init_train_L_Y2);
                        flag_gprMdl_x14_int2=1;
                    else
                        all_train_L_14_int2_X=[last_train_14_X2; train_L_X;];
                        all_train_L_14_int2_Y=[last_train_14_Y2; train_L_Y;];
                        gprMdl_x14_int2 = updateGPRMdl(gprMdl_x14_int2,all_train_L_14_int2_X, all_train_L_14_int2_Y);
                    end
                        last_train_14_X2=train_L_X;
                        last_train_14_Y2=train_L_Y;                      
                    
                    if last_start ~= 0
                        Iuse_start2=last_start
                        result = myGRP_onlyx_limitfuture(last_start,2, gprMdl_x14_int2,200);
                        my_result2=result(end)
                        result=[2; result;];
                    end
                end
                
                
            elseif in_step == 25
                if intent == 1
                    if flag_gprMdl_x25_int1 == 0
                        init_train_L_X1=train_L_X;
                        init_train_L_Y1=train_L_Y;
                        gprMdl_x25_int1 = fitrgp(init_train_L_X1,init_train_L_Y1);
                        flag_gprMdl_x25_int1=1;
                    else
                        all_train_L_25_int1_X=[last_train_25_X1; train_L_X;];
                        all_train_L_25_int1_Y=[last_train_25_Y1; train_L_Y;];
                        gprMdl_x25_int1= updateGPRMdl(gprMdl_x25_int1,all_train_L_25_int1_X, all_train_L_25_int1_Y);
                    end
                        last_train_25_X1=train_L_X;
                        last_train_25_Y1=train_L_Y;   
                        
                    if last_start ~= 0
                        Iuse_start1=last_start
                        result = myGRP_onlyx_limitfuture(last_start, 1, gprMdl_x25_int1,200);
                        my_result1=result(end)
                        result=[1; result;];
                    end
                else
                    if flag_gprMdl_x25_int2 == 0
                        init_train_L_X2=train_L_X;
                        init_train_L_Y2=train_L_Y;
                        gprMdl_x25_int2 = fitrgp(init_train_L_X2,init_train_L_Y2);
                        flag_gprMdl_x25_int2=1;
                    else
                        all_train_L_25_int2_X=[last_train_25_X2; train_L_X;];
                        all_train_L_25_int2_Y=[last_train_25_Y2; train_L_Y;];
                        gprMdl_x25_int2 = updateGPRMdl(gprMdl_x25_int2,all_train_L_25_int2_X, all_train_L_25_int2_Y);
                    end
                        last_train_25_X2=train_L_X;
                        last_train_25_Y2=train_L_Y;                      
                    
                    if last_start ~= 0
                        Iuse_start2=last_start
                        result = myGRP_onlyx_limitfuture(last_start,2, gprMdl_x25_int2,200);
                        my_result2=result(end)
                        result=[2; result;];
                    end
                end
                
            elseif in_step == 36
                if intent == 1
                    if flag_gprMdl_x36_int1 == 0
                        gprMdl_x36_int1 = fitrgp(train_L_X,train_L_Y);
                        flag_gprMdl_x36_int1=1;
                    else
                        all_train_L_36_int1_X=[last_train_36_X1; train_L_X;];
                        all_train_L_36_int1_Y=[last_train_36_Y1; train_L_Y;];
                        gprMdl_x36_int1= updateGPRMdl(gprMdl_x36_int1,all_train_L_36_int1_X, all_train_L_36_int1_Y);
                    end
                        last_train_36_X1=train_L_X;
                        last_train_36_Y1=train_L_Y;   
                        
                    if last_start ~= 0
                        Iuse_start1=last_start
                        result = myGRP_onlyx_limitfuture(last_start, 1, gprMdl_x36_int1,200);
                        my_result1=result(end)
                        result=[1; result;];
                    end
                else
                    if flag_gprMdl_x36_int2 == 0
                        init_train_L_X2=train_L_X;
                        init_train_L_Y2=train_L_Y;
                        gprMdl_x36_int2 = fitrgp(init_train_L_X2,init_train_L_Y2);
                        flag_gprMdl_x36_int2=1;
                    else
                        all_train_L_36_int2_X=[last_train_36_X2; train_L_X;];
                        all_train_L_36_int2_Y=[last_train_36_Y2; train_L_Y;];
                        gprMdl_x36_int2 = updateGPRMdl(gprMdl_x36_int2,all_train_L_36_int2_X, all_train_L_36_int2_Y);
                    end
                        last_train_36_X2=train_L_X;
                        last_train_36_Y2=train_L_Y;                      
                    
                    if last_start ~= 0
                        Iuse_start2=last_start
                        result = myGRP_onlyx_limitfuture(last_start,2, gprMdl_x36_int2,200);
                        my_result2=result(end)
                        result=[2; result;];
                    end
                end 
                
                
            end
            
            if result ~= 0
%                 all_result=[all_result; [result last_start*ones(size(result,1),1)];];
                fwrite(t_client_EMG_local,[result.'],'double');%写入数字数据，每次发送360个double
            end
            

            
            last_in_step=in_step;
            
        end       
        toc
    end

end


disp('Transport is done!');
fclose(t_client_EMG_local); 



