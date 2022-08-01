
IP_remote = "192.168.11.1"; 
port_remote = 5000;
IP_local = "192.168.11.2"; 

port_local = 5000;
Role = 'client';

tcp_rece = tcpip(IP_remote,port_remote,...
                'NetworkRole',Role,...
                'LocalPort',port_local,...
                'TimeOut',20,...
                'InputBufferSize',8192);
            
str_all = [];
fopen(tcp_rece);
for q = 1:2000
    pause(0.01);this_recv=[];    
    recv2 = [];
    flagg=0;   
    pass=0;
    q
while tcp_rece.BytesAvailable > 0
%    str_rece = fscanf(tcp_rece);
%     str_rece=fread(tcp_rece);
  str_rece=fread(tcp_rece,tcp_rece.BytesAvailable,'float');
  
   recv1=dec2hex(str_rece,2);
   
     recv2 = [];
    flagg=0;   

    pass=0;
    for i = 1:length(recv1) 
        if pass == 0;
           if i <= length(recv1)-7 && sum(recv(i:(i+7)) == 122) == 8;
                pass = 7;
                flagg=flagg+1;
            else
                recv2 = [recv2,recv1(i,:)]; %小端模式
            end
        else 
            pass = pass -1;
        end
    end  
   hex2num(recv2(1*16+1:1*16+16));
%    str_rece2=str2num(str_rece)
%    this_recv=[this_recv str_rece2];
%    str_all = [str_all str_rece2];
end
end

fclose(tcp_rece); 

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