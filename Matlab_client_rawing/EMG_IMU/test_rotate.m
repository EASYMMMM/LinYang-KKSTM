clear all;
clc
close all;

% ori=readmatrix('20220106_200058_lpms1.csv');
% del1=find(ori(:,2) == 1);
% new_ori=ori(del1:end,:);


R = 0.005;  % 测量噪音方差矩阵
kalmanx = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalmany = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalmanz = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter

all_data=load('up.mat').all_data;
nmb1=find(all_data(:,19) == 111);
nmb2=find(all_data(:,19) == 222);
nmb3=find(all_data(:,19) == 333);




data1=all_data(nmb1,:);
data2=all_data(nmb2,:);
data3=all_data(nmb3,:);

% figure;plot(data1(:,3))

% data1=new_ori;

all_new=[];
all_velo=[];
init_v=[0; 0; 0;];
filter_A=[];
for leno = 1:size(data1,1) 
    this = data1(leno, :);
%     bias_x=sum(data1(1:100,3))/100;
%     bias_y=sum(data1(1:100,4))/100;
%     bias_z=sum(data1(1:100,5))/100;
    %%
    a=this(15);
    b=this(14);
    y=this(13);
    raw_Ax=this(3)-bias_x;
    raw_Ay=this(4)-bias_y;
    raw_Az=this(5)-bias_z;
    Ax = kalmanx(raw_Ax);
    Ay = kalmany(raw_Ay);
    Az = kalmanz(raw_Az);
    filter_A=[filter_A [Ax; Ay; Az;]];
    
    A=[Ax; Ay; Az;];
    Rz=[[cosd(a) -sind(a) 0]; [sind(a) cosd(a) 0]; [0 0 1];];
    Ry=[[cosd(b) 0 sind(b)]; [0 1 0]; [-sind(b) 0 cosd(b)];];
    Rx=[[1 0 0]; [0 cosd(y) -sind(y)]; [0 sind(y) cosd(y)];];
    R=Rz*Ry*Rx;
    new_A=R*A;
    all_new=[all_new new_A];
 %%   
    init_v=init_v+new_A;
    all_velo=[all_velo init_v];
    
    
    
    
end

figure(1);
plot(data1(:,3)); hold on; plot(filter_A(1,:));

figure(2);
plot(all_velo(1,:));


