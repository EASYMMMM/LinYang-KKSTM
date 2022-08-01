
close all;clear;clc;

t_client_EMG_local=tcpip('localhost',20000,'NetworkRole','client','TimeOut',200);%与本地主机建立连接，端口号为30000，作为客户机连接。
t_client_EMG_local.OutputBuffersize=100000;
disp(['未打开！',datestr(now)])
fopen(t_client_EMG_local);%与一个服务器建立连接，直到建立完成返回，否则报错。
disp(['已打开！',datestr(now)])


direction=1;

MY_COUNT=0; judge2=5000; judge3=15000;
while MY_COUNT<500
    pause(0.02);
    MY_COUNT=MY_COUNT+1;
    y_new=MY_COUNT;
    fwrite(t_client_EMG_local,[88888.888,y_new,direction],'double');%写入数字数据，每次发送360个double
end


disp('Transport is done!');
fclose(t_client_EMG_local); 



