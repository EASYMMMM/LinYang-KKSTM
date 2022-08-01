clc;clear;

t_server_self=tcpip('0.0.0.0',30000,'NetworkRole','server','TimeOut',200);%与第一个请求连接的客户机建立连接，端口号为30000，类型为服务器。
t_server_self.InputBuffersize=100000;
disp(['未打开！',datestr(now)])
fopen(t_server_self);%打开服务器，直到建立一个TCP连接才返回；
disp(['已打开！',datestr(now)])
%get(t_server);
data_all = [];count = 0;
%代表那个大循环
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