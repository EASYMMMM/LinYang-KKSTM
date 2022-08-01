clc;clear;

t_client_EMG_local=tcpip('localhost',30000,'NetworkRole','client');%与本地主机建立连接，端口号为30000，作为客户机连接。
t_client_EMG_local.OutputBuffersize=100000;
disp(['未打开！',datestr(now)])
fopen(t_client_EMG_local);%与一个服务器建立连接，直到建立完成返回，否则报错。
disp(['已打开！',datestr(now)])
data_send=sin((1:360)/180 * pi);%发送的数字数据。
% pause(1);%等待连接稳定，随意设置。
for i = 1:60
    pause(0.5);
    fwrite(t_client_EMG_local,data_send,'double');%写入数字数据，每次发送360个double
end
disp(['已关闭！',datestr(now)])
pause(1);
fclose(t_client_EMG_local)
