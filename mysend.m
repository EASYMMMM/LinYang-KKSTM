clc;clear;

t_client_EMG_local=tcpip('localhost',30000,'NetworkRole','client');%�뱾�������������ӣ��˿ں�Ϊ30000����Ϊ�ͻ������ӡ�
t_client_EMG_local.OutputBuffersize=100000;
disp(['δ�򿪣�',datestr(now)])
fopen(t_client_EMG_local);%��һ���������������ӣ�ֱ��������ɷ��أ����򱨴�
disp(['�Ѵ򿪣�',datestr(now)])
data_send=sin((1:360)/180 * pi);%���͵��������ݡ�
% pause(1);%�ȴ������ȶ����������á�
for i = 1:60
    pause(0.5);
    fwrite(t_client_EMG_local,data_send,'double');%д���������ݣ�ÿ�η���360��double
end
disp(['�ѹرգ�',datestr(now)])
pause(1);
fclose(t_client_EMG_local)
