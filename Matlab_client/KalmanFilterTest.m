% 读取数据
clc;
clear;
filename = 'D:\LiuXiaodong\CodeMatlab\KalmanFilter\force.txt';
fileID = fopen(filename);
data_original = textscan(fileID,'%f %f %f');
fclose(fileID);

% Kalman
R = 0.005;  % 测量噪音方差矩阵
kalman = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
TITLE = {'X方向的力','Y方向的力','Z方向的力'}
for j = 1:3
    data_filtered = zeros(size(data_original{1,j},1),1);
    for i = 1:size(data_original{1,j},1)
        data_filtered(i) = kalman(data_original{1,j}(i));
%         disp([num2str(i),'  ',num2str(data_filtered(i))]);
    end
    figure(j);
%     plot(data_filtered);hold on;
%     plot(data_original{1,j});

    plot(data_filtered,'ro-');hold on;
    plot(data_original{1,j},'bo-');
    title(TITLE{j});
    legend('After Kalman Filter','Before Kalman Filter')
end