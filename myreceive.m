clc;clear;

t_server_self=tcpip('0.0.0.0',30000,'NetworkRole','server','TimeOut',200);%���һ���������ӵĿͻ����������ӣ��˿ں�Ϊ30000������Ϊ��������
t_server_self.InputBuffersize=100000;
disp(['δ�򿪣�',datestr(now)])
fopen(t_server_self);%�򿪷�������ֱ������һ��TCP���Ӳŷ��أ�
disp(['�Ѵ򿪣�',datestr(now)])
%get(t_server);
data_all = [];count = 0;
%�����Ǹ���ѭ��
all_dt=[];
while 1
    pause(0.01)
    
    if  t_server_self.BytesAvailable>0
     data_recv = fread(t_server_self,t_server_self.BytesAvailable/8,'double');%    disp(size(data_recv));
    count = count + 1;
    which_head=find(88887<=data_recv);
    which_head2=which_head(end);
    this_frame=data_recv(which_head2:end)
    
    data_all(:,count) = this_frame;
    end
     
end
fclose(t_server_self);