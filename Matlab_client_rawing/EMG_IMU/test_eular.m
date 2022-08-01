clear all;
clc
close all;

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

all_data=load('up1_d_0108.mat').all_data;
nmb1=find(all_data(:,19) == 111);
nmb2=find(all_data(:,19) == 222);
nmb3=find(all_data(:,19) == 333);
data1=all_data(nmb1,:);
data2=all_data(nmb2,:);
data3=all_data(nmb3,:);

all_a=[]; all_b=[]; all_y=[];
del1=find(data1(:,2) == 1);
del2=find(data2(:,2) == 1);
del3=find(data3(:,2) == 1);
data1=data1(del1:end,:);
data2=data2(del2:end,:);
data3=data3(del3:end,:);

l1=size(data1,1);
l2=size(data2,1);
l3=size(data3,1);
lll=min([l1,l2,l3]);

filter_A=[];
for leno = 1:lll
    this1 = data1(leno, :);
    this2 = data2(leno, :);
    this3 = data3(leno, :);
    %%
%     a=this1(15);
%     b=this1(14);
%     y=this1(13);

%     Ax = kalmanx(raw_Ax);
%     Ay = kalmany(raw_Ay);
%     Az = kalmanz(raw_Az);
%     filter_A=[filter_A [Ax; Ay; Az;]];

    R1 = rotate_matrix(this1);
    R2 = rotate_matrix(this2);
    R3 = rotate_matrix(this3);
    R31 = R1.'*R3;
    R23 = R3.'*R2;
    [b,a,y] = inverse_angle(R23);
    all_a=[all_a a];
    all_b=[all_b b];
    all_y=[all_y y];

end
figure(1)
plot(all_y); hold on;
% figure(2)
plot(all_b); hold on;
% figure(3)
plot(all_a)
legend('绕x轴转的角度','绕y轴转的角度','绕z轴转的角度')

