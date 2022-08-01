
% IP_remote = "192.168.1.1"; 
% port_remote = 5000;
% IP_local = "192.168.1.2"; 
% 
% port_local = 5000;
% Role = 'client';
% 
% tcp_rece = tcpip(IP_remote,port_remote,...
%                 'NetworkRole',Role,...
%                 'LocalPort',port_local,...
%                 'TimeOut',20,...
%                 'InputBufferSize',8192);
%             
% str_all = [];
% fopen(tcp_rece);
% while tcp_rece.BytesAvailable > 0
%    str_rece = fscanf(tcp_rece); 
%    str_all = [str_all str_rece];
% end
% fclose(tcp_rece); 



close all;clear;clc;
warning('off')

draw_points=200;FLAG=0;predict=[];
wrong=0;how_many_points_0=[];kinds=12;how_many_points_1=[];
how_many_points_2=[];how_many_heads_total=0;all_middle=[];
how_many_points_3=[];how_many_points_4=[];how_many_points_5=[];how_many_points_11=[];
how_many_points_6=[];how_many_points_7=[];how_many_points_8=[];how_many_points_9=[];how_many_points_10=[];
    recv_once_0=[];recv_once_1=[];recv_once_2=[];recv_once_3=[];recv_once_4=[];recv_once_5=[];recv_once_6=[];
     recv_once_6=[];recv_once_7=[];recv_once_8=[];recv_once_9=[];recv_once_10=[];recv_once_11=[];recv_once_12=[];current_9xyz=[];
all_9xyz=[];final_EMGxyz=[];
global b emg_show handle flag
% emg_show  = zeros(1000,1);
% figure(1);
% handle = plot(b);
% drawnow;
flag = 0;count_which_matrix=0;

global READY
READY=0;

global each_update_16
each_update_16=[];
% %% 设置连接参数，要连接的地址为127.0.0.1(即本地主机)，端口号为5174，作为客户机连接。
% Client=tcpip('127.0.0.1',5000,'NetworkRole','client');
% 
% %% 建立连接，建立完成后进行下一步，否则报错
% fopen(Client);%与一个服务器建立连接，直到建立完成返回，否则报错。
% sprintf('what is it?')


%% 发送字符串，pause（1）要不要都可以
% sendtxt = 'hello hello';
% fprintf(Client,sendtxt);
total_rece=[];count2=0;count_x=0;
total_rece_0=[];total_rece_1=[];total_rece_2=[];total_rece_6=[];ALL_LEN_OF_RECV2=[];
total_rece = [];total_head=0;total_rece_3=[];total_rece_4=[];total_rece_5=[];
global all_len_fun
all_len=[];all_len_fun=[];
%%
global DATA;
DATA=[];global all_len_fun_if
all_len_fun_if=[];
% CHANGE THIS TO THE IP OF THE COMPUTER RUNNING THE TRIGNO CONTROL UTILITY
HOST_IP = '172.31.75.32';
%%
%This example program communicates with the Delsys SDK to stream 16
%channels of EMG data and 48 channels of ACC data.



%% Create the required objects

%Define number of sensors
NUM_SENSORS = 16;

%handles to all plots
global plotHandlesEMG;
plotHandlesEMG = zeros(NUM_SENSORS,1);
global plotHandlesACC;
plotHandlesACC = zeros(NUM_SENSORS*3, 1);
global rateAdjustedEmgBytesToRead;

%TCPIP Connection to stream EMG Data
interfaceObjectEMG = tcpip(HOST_IP,50041);
interfaceObjectEMG.InputBufferSize = 6400;

%TCPIP Connection to stream ACC Data
interfaceObjectACC = tcpip(HOST_IP,50042);
interfaceObjectACC.InputBufferSize = 6400;

%TCPIP Connection to communicate with SDK, send/receive commands
commObject = tcpip(HOST_IP,50040);

%Timer object for drawing plots.
t = timer('Period', .1, 'ExecutionMode', 'fixedSpacing', 'TimerFcn', {@updatePlots, plotHandlesEMG});
global data_arrayEMG
data_arrayEMG = [];
global data_arrayACC
data_arrayACC = [];

%% Set up the plots
axesHandlesEMG = zeros(NUM_SENSORS,1);
axesHandlesACC = zeros(NUM_SENSORS,1);

%initiate the EMG figure
figureHandleEMG = figure('Name', 'EMG Data','Numbertitle', 'off',  'CloseRequestFcn', {@localCloseFigure, interfaceObjectEMG, interfaceObjectACC, commObject, t});
set(figureHandleEMG, 'position', [50 200 750 750])

for i = 1:NUM_SENSORS
    axesHandlesEMG(i) = subplot(4,4,i);

    plotHandlesEMG(i) = plot(axesHandlesEMG(i),0,'-y','LineWidth',1);

    set(axesHandlesEMG(i),'YGrid','on');
    %set(axesHandlesEMG(i),'YColor',[0.9725 0.9725 0.9725]);
    set(axesHandlesEMG(i),'XGrid','on');
    %set(axesHandlesEMG(i),'XColor',[0.9725 0.9725 0.9725]);
    set(axesHandlesEMG(i),'Color',[.15 .15 .15]);
    set(axesHandlesEMG(i),'YLim', [-.005 .005]);
    set(axesHandlesEMG(i),'YLimMode', 'manual');
    set(axesHandlesEMG(i),'XLim', [0 2000]);
    set(axesHandlesEMG(i),'XLimMode', 'manual');
    
    if(mod(i, 4) == 1)
        ylabel(axesHandlesEMG(i),'V');
    else
        set(axesHandlesEMG(i), 'YTickLabel', '')
    end
    
    if(i >12)
        xlabel(axesHandlesEMG(i),'Samples');
    else
        set(axesHandlesEMG(i), 'XTickLabel', '')
    end
    
    title(sprintf('EMG %i', i)) 
end

%initiate the ACC figure
figureHandleACC = figure('Name', 'ACC Data', 'Numbertitle', 'off', 'CloseRequestFcn', {@localCloseFigure, interfaceObjectEMG, interfaceObjectACC, commObject, t});
set(figureHandleACC, 'position', [850 200 750 750]);

for i= 1:NUM_SENSORS
    axesHandlesACC(i) = subplot(4, 4, i);
    hold on
    plotHandlesACC(i*3-2) = plot(axesHandlesACC(i), 0, '-y', 'LineWidth', 1);    
    plotHandlesACC(i*3-1) = plot(axesHandlesACC(i), 0, '-y', 'LineWidth', 1);   
    plotHandlesACC(i*3) = plot(axesHandlesACC(i), 0, '-y', 'LineWidth', 1);
    hold off 
    
    set(plotHandlesACC(i*3-2), 'Color', 'r')
    set(plotHandlesACC(i*3-1), 'Color', 'b')
    set(plotHandlesACC(i*3), 'Color', 'g')    
    set(axesHandlesACC(i),'YGrid','on');
    %set(axesHandlesACC(i),'YColor',[0.9725 0.9725 0.9725]);
    set(axesHandlesACC(i),'XGrid','on');
    %set(axesHandlesACC(i),'XColor',[0.9725 0.9725 0.9725]);
    set(axesHandlesACC(i),'Color',[.15 .15 .15]);
    set(axesHandlesACC(i),'YLim', [-8 8]);
    set(axesHandlesACC(i),'YLimMode', 'manual');
    set(axesHandlesACC(i),'XLim', [0 2000/13.5]);
    set(axesHandlesACC(i),'XLimMode', 'manual');
    
    if(i > 12)
        xlabel(axesHandlesACC(i),'Samples');
    else
        set(axesHandlesACC(i), 'XTickLabel', '');
    end
    
    if(mod(i, 4) == 1)
        ylabel(axesHandlesACC(i),'g');
    else
        set(axesHandlesACC(i) ,'YTickLabel', '')
    end
    
    title(sprintf('ACC %i', i)) 

end

%%Open the COM interface, determine RATE

fopen(commObject);

pause(1);
fread(commObject,commObject.BytesAvailable);
fprintf(commObject, sprintf(['RATE 2000\r\n\r']));
pause(1);
fread(commObject,commObject.BytesAvailable);
fprintf(commObject, sprintf(['RATE?\r\n\r']));
pause(1);
data = fread(commObject,commObject.BytesAvailable);

emgRate = strtrim(char(data'));
if(strcmp(emgRate, '1925.926'))
    rateAdjustedEmgBytesToRead=1664;
else 
    rateAdjustedEmgBytesToRead=1728;
end


%% Setup interface object to read chunks of data
% Define a callback function to be executed when desired number of bytes
% are available in the input buffer
 bytesToReadEMG = rateAdjustedEmgBytesToRead;
 interfaceObjectEMG.BytesAvailableFcn = {@localReadAndPlotMultiplexedEMG,plotHandlesEMG,bytesToReadEMG};
 interfaceObjectEMG.BytesAvailableFcnMode = 'byte';
 interfaceObjectEMG.BytesAvailableFcnCount = bytesToReadEMG;
 
 bytesToReadACC = 384;
interfaceObjectACC.BytesAvailableFcn = {@localReadAnPlotMultiplexedACC, plotHandlesACC, bytesToReadACC};
interfaceObjectACC.BytesAvailableFcnMode = 'byte';
interfaceObjectACC.BytesAvailableFcnCount = bytesToReadACC;

% drawnow
% start(t);

%% 
% Open the interface object
try
    fopen(interfaceObjectEMG);
    fopen(interfaceObjectACC);
catch
    localCloseFigure(figureHandleACC,1 ,interfaceObjectACC, interfaceObjectEMG, commObject, t);
    delete(figureHandleEMG);
    error('CONNECTION ERROR: Please start the Delsys Trigno Control Application and try again');
end

%%
% Send the commands to start data streaming
fprintf(commObject, sprintf(['START\r\n\r']));
disp('run into the loop')
old=[];
sss=0;
count2=0;
total=[];START=0;   
all_combine=[];all_delta_wan_y=[];
% [-5.84131140848809e-05,1.72206388957788e-05,-3.74196484679920e-05,-5.98080815487738e-05,-6.46232878384343e-05,-5.06079718533420e-05,-9.73183698356341e-06];
% =zeros(1,9);
bias=[-5.65280921398098e-05,1.66940316482401e-05,-4.02088271320347e-05,-5.61586751568069e-05,-6.47374386382968e-05,-5.02930837581653e-05,-1.03957137825998e-05,-1.10032264596377e-05,-3.71385302006891e-05];
% [-5.73264426787071e-05,1.71024708678568e-05,-3.66252162620216e-05,-5.89599011197719e-05,-6.50315308334998e-05,-5.03726010623036e-05,-1.01644666404895e-05,-1.37446684535868e-05,-4.02487479223647e-05];
% bias=[-5.61904605277690e-05,1.72309199767471e-05,-3.88568792333475e-05,-5.71291390760881e-05,-6.36873786758940e-05,-4.97517192231279e-05,-1.03198857677896e-05,-1.09548321308460e-05,-3.64994955423354e-05];

% bias=[-5.82286876606189e-05,1.71419019774182e-05,-3.46533367015892e-05,-6.06744803161353e-05,-6.49713439261212e-05,-5.07107625050614e-05,-9.89152311001298e-06]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%========================


t_client_EMG_local=tcpip('localhost',30000,'NetworkRole','client','TimeOut',200);%与本地主机建立连接，端口号为30000，作为客户机连接。
t_client_EMG_local.OutputBuffersize=100000;
disp(['未打开！',datestr(now)])
fopen(t_client_EMG_local);%与一个服务器建立连接，直到建立完成返回，否则报错。
disp(['已打开！',datestr(now)])

x_all = [];all_intension=[];
y_all = [];all_9xyz=[];
global each_update_16 flag
direction=2.5; j=0;
B= [0.0005371697748121, 0.001074339549624,0.0005371697748121];
A= [1,    -1.93338022588,   0.9355289049792];
            
            
    all_dt=[];  loop=0;  init_current_all=[]; 
                
            y_EMG_afterfilter = zeros(2,9);
            x_EMG_now=zeros(2,9);
            all_intension=x_EMG_now;
            

while 1
   if sum(flag)~=0
       disp(flag);
      break; 
   end    
end
disp('Delsys is open!');
all_9xyz=ones(300000,9);
from_EMG=1;to_EMG=0;
all_pass0=[];

while 1
    tic
    pause(0.002);
    if ~isempty(each_update_16)
        current_all=[];
        loop=loop+1
        direction= sin(loop/100 * 2 * pi); 
        temp=each_update_16;
        size(temp)
        for each =1:9
            if each == 3
                each = 11;
            end
            if each == 9
                each = 16;
            end          
            
            each_tongdao=temp(each:16:end);
            current_all=[current_all each_tongdao];
        end 

%         each_update_16=each_update_16(end-16+1:end);
        each_update_16=[];
        to_EMG=to_EMG+size(current_all,1);
        all_9xyz(from_EMG:to_EMG,:)=current_all;
        from_EMG=from_EMG+size(current_all,1);
        
         intension=abs(mean(current_all)-bias);
        all_intension=[all_intension; intension;];
        
       x_EMG_now=[x_EMG_now(end-1:end,:); intension;];
            y_new=[0 0 0 0 0 0 0 0 0];
    for i = 1:length(B)
          y_new = y_new + (B(i) * x_EMG_now(end-i+1,:));
    end
    for i = 2:length(B)
          y_new = y_new - (A(i) * y_EMG_afterfilter(end-i+2,:));
    end
    y_EMG_afterfilter=[y_EMG_afterfilter; y_new;];
        
    
  %%  
  nmb=current_all;
  pass_0_this=[];
    for p = 1:size(nmb,2)
        this_col=nmb(:,p);
        count_pass0=0;
        for each_inthis = 1:length(this_col)-1
            first=this_col(each_inthis);
            second=this_col(each_inthis+1);
            if first*second<0
                count_pass0=count_pass0+1;
            end
        end
        pass_0_this=[pass_0_this count_pass0];
    end
        
  
    
        fwrite(t_client_EMG_local,[88888.888,[y_new pass_0_this],direction],'double');%写入数字数据，每次发送360个double
    end
    toc
end


disp('Transport is done!');
fclose(t_client_EMG_local); 

who=find(all_9xyz(:,3)~=1);
use=all_9xyz(who,:);
who2=find(use(:,3)~=0);
use2=all_9xyz(who2,:);
jj=floor(size(use2,1)/3)
use3=use2(1:jj,:);
bias=mean(use3,1)
% jj=use-bias;
%     Hd=butter15;
%     after=filter(Hd,all_intension);
%     figure;plot(all_intension(:,2));hold on; plot(y_EMG_afterfilter(:,2)); hold on; plot(after(:,2));
%     legend('ori','first','second')



% bias=mean(use);

%  clust = parcluster('local');
%  job1 = createJob(clust); %开启一个job
%  disp('saving------');
%  temp = final_EMGxyz;
%  createTask(job1,@mytxt,1,{temp});%再给job1分配一个‘mytxt’的task
%  submit(job1);
 



%% Implement the bytes available callback
%The localReadandPlotMultiplexed functions check the input buffers for the
%amount of available data, mod this amount to be a suitable multiple.

%Because of differences in sampling frequency between EMG and ACC data, the
%ratio of EMG samples to ACC samples is 13.5:1

%We use a ratio of 27:2 in order to keep a whole number of samples.  
%The EMG buffer is read in numbers of bytes that are divisible by 1728 by the
%formula (27 samples)*(4 bytes/sample)*(16 channels)
%The ACC buffer is read in numbers of bytes that are divisible by 384 by
%the formula (2 samples)*(4 bytes/sample)*(48 channels)
%Reading data in these amounts ensures that full packets are read.  The 
%size limits on the dataArray buffers is to ensure that there is always one second of
%data for all 16 sensors (EMG and ACC) in the dataArray buffers
function localReadAndPlotMultiplexedEMG(interfaceObjectEMG, ~,~,~, ~)
global rateAdjustedEmgBytesToRead;
bytesReady = interfaceObjectEMG.BytesAvailable;
bytesReady = bytesReady - mod(bytesReady, rateAdjustedEmgBytesToRead);%%1664

if (bytesReady == 0)
    return
end
% global data_arrayEMG
data = cast(fread(interfaceObjectEMG,bytesReady), 'uint8');
data = typecast(data, 'single');

global each_update_16 flag

global data_arrayEMG
flag = data;

if size(each_update_16,1) < rateAdjustedEmgBytesToRead*19
    each_update_16 = [each_update_16; data];
else
    each_update_16 = [each_update_16(size(data,1) + 1:size(each_update_16, 1));data];
end


if(size(data_arrayEMG, 1) < rateAdjustedEmgBytesToRead*19)
    data_arrayEMG = [data_arrayEMG; data];
else
    data_arrayEMG = [data_arrayEMG(size(data,1) + 1:size(data_arrayEMG, 1));data];
end
end


function localReadAnPlotMultiplexedACC(interfaceObjectACC, ~, ~, ~, ~)

bytesReady = interfaceObjectACC.BytesAvailable;
bytesReady = bytesReady - mod(bytesReady, 384);

if(bytesReady == 0)
    return
end
global data_arrayACC
data = cast(fread(interfaceObjectACC, bytesReady), 'uint8');
data = typecast(data, 'single');





if(size(data_arrayACC, 1) < 7296)
    data_arrayACC = [data_arrayACC; data];
else
    data_arrayACC = [data_arrayACC(size(data, 1) + 1:size(data_arrayACC, 1)); data];
end

end

%% Update the plots
%This timer callback function is called on every tick of the timer t.  It
%demuxes the dataArray buffers and assigns that channel to its respective
%plot.
function updatePlots(obj, Event,  tmp)
% global data_arrayEMG
% global plotHandlesEMG
% 
% for i = 1:size(plotHandlesEMG, 1) 
%     data_ch = data_arrayEMG(i:16:end);      
%     set(plotHandlesEMG(i), 'Ydata', data_ch)
% end
% 
% 
% global data_arrayACC
% global plotHandlesACC
% for i = 1:size(plotHandlesACC, 1)
%     data_ch = data_arrayACC(i:48:end);
%     set(plotHandlesACC(i), 'Ydata', data_ch)
% end
% drawnow

end


%% Implement the close figure callback
%This function is called whenever either figure is closed in order to close
%off all open connections.  It will close the EMG interface, ACC interface,
%commands interface, and timer object
function localCloseFigure(figureHandle,~,interfaceObject1, interfaceObject2, commObject, t)

%% 
% Clean up the network objects
if isvalid(interfaceObject1)
    fclose(interfaceObject1);
    delete(interfaceObject1);
    clear interfaceObject1;
end
if isvalid(interfaceObject2)
    fclose(interfaceObject2);
    delete(interfaceObject2);
    clear interfaceObject2;
end



if isvalid(t)
   stop(t);
   delete(t);
end

if isvalid(commObject)
    fclose(commObject);
    delete(commObject);
    clear commObject;
end

%% 
% Close the figure window
delete(figureHandle);
end
