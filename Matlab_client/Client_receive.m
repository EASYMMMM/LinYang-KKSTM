close all;clear;clc;
now=1;
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

data_all = [];count_right = 0;
for q = 1:2000
    pause(0.01);this_recv=[];    
    recv2 = [];
    flagg=0;   
    pass=0;
    q
     if  t_server.BytesAvailable>0
%              t_server.BytesAvailable % 代表缓冲区里累积了多少的字节数的数据/8

%     data_recv = fread(t_server,t_server.BytesAvailable/8,'double');%从缓冲区读取数字数据

    data_recv = fread(t_server,t_server.BytesAvailable/8,'double');% 第二个参数代表 要从缓冲区里读取多少个 double
%     disp(size(data_recv));
    count_right = count_right + 1;
    which_head=find(88887<=data_recv);
%     [o,which_head]=min(abs(data_recv-888.888));
    
    which_head2=which_head(end);
    this_frame=data_recv(which_head2:end);
    if length(this_frame)<16
        this_frame=[this_frame; [0 0 0 0 0 0 0].'];
    end
    data_all(:,count_right) = this_frame;
     end
     
     
end

fclose(t_server); 

% IP_local  = "10.0.8.254"; 
% port_remote = 5000;
% IP_remote = "10.11.0.1"; 
% port_local = 5000;
% Role = 'server';
% 
% tcp_send = tcpip(IP_remote,port_remote,...
%                 'NetworkRole',Role,...
%                 'LocalPort',port_local,...
%                 'TimeOut',100,...
%                 'OutputBufferSize',8192);
%             
% fopen(tcp_send);
% disp('tcp_send is open!');
% x_all = [];
% for i = 1:1000
%     x = sin(i/1000 * 2 * pi);
%     x_all = [x_all num2str(x)];
%     fprintf(tcp_send,x);
%     pause(0.1);    
% end
% fclose(tcp_send); 



% ipA = '10.0.8.254'; 
% portA = 8080;
% ipB = '10.11.0.1';
% portB = 8080;
% handles.udpA = udp(ipB,portB,'LocalPort',portA);
% set(handles.udpA,'OutputBufferSize',8192);
% set(handles.udpA,'TimeOut',100);
% fopen(handles.udpA);
% for t = 1:100
%     x = sin(t);
%     set(handles.edit1,'String',x);
%     v1=str2double(get(handles.edit1,'string'));
%     w1(t)=v1;
%     fprintf(handles.udpA,v1);
%     pause(0.1);
% end