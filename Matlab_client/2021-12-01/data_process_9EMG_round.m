close all;clear;clc;


figure(3);
t=1:1000;
p=polyfit(t,round1(:,8),20);
yi=polyval(p,t);
plot(t,round1(:,8),':o',t,yi,'-*'); hold on;
plot(slope*10)

slope=[];
for i = 1:length(yi)-1
    this = yi(i);
    next=yi(i+1);
    delta=next-this;
    slope=[slope delta];
end
sorted_slope=sort(abs(slope));
threhold=sorted_slope(800)
which_want=find(abs(slope) > threhold);
want=yi(which_want);


t=1:1000;
p=polyfit(t,round1(:,7),20);
yi=polyval(p,t);


slope=[];
for i = 1:length(yi)-1
    this = yi(i);
    next=yi(i+1);
    delta=next-this;
    slope=[slope delta];
end
sorted_slope=sort(abs(slope));
threhold=sorted_slope(800)
which_want=find(abs(slope) > threhold);
want=yi(which_want);

figure(4);
plot(t,round1(:,7),':o',t,yi,'-*'); hold on;
plot(slope*10)


%%
down_fix0=down(89:479,13:21);% EMG
up2_fix0=up2(120:530,13:21);
up1_fix0=up1(80:420,13:21);

forward_fix0=forward(96:307,13:21); 
back_fix0=back(96:307,13:21);
right_fix0=right(90:330,13:21);
left_fix0=left(52:270,13:21);


down_cor=down(89:479,7:9);% xzy
up2_cor=up2(120:530,7:9);
up1_cor=up1(80:420,7:9);

forward_cor=forward(96:307,7:9); 
back_cor=back(96:307,7:9);
right_cor=right(90:330,7:9);
left_cor=left(52:270,7:9);





up1_fix=mapminmax(up1_fix0.',0,1).';
forward_fix=mapminmax(forward_fix0.',0,1).';
back_fix=mapminmax(back_fix0.',0,1).';
right_fix=mapminmax(right_fix0.',0,1).';
down_fix=mapminmax(down_fix0.',0,1).';
left_fix=mapminmax(left_fix0.',0,1).';
% 找到拮抗肌组
EMG_train=[[forward_fix(1:end-1,1)-forward_fix(1:end-1,2 ) forward_fix(1:end-1,3 )-forward_fix(1:end-1,4 ) forward_fix(1:end-1,5 )-forward_fix(1:end-1,7 ) forward_fix(1:end-1,8 )-forward_fix(1:end-1,9 ) forward_fix(1:end-1,6 )]; ...
[back_fix(1:end-1,1)-back_fix(1:end-1,2 ) back_fix(1:end-1,3 )-back_fix(1:end-1,4 ) back_fix(1:end-1,5 )-back_fix(1:end-1,7 ) back_fix(1:end-1,8 )-back_fix(1:end-1,9 ) back_fix(1:end-1,6 )]; ...
[right_fix(1:end-1,1)-right_fix(1:end-1,2 ) right_fix(1:end-1,3 )-right_fix(1:end-1,4 ) right_fix(1:end-1,5 )-right_fix(1:end-1,7 ) right_fix(1:end-1,8 )-right_fix(1:end-1,9 ) right_fix(1:end-1,6 )]; ...
[left_fix(1:end-1,1 )-left_fix(1:end-1,2 ) left_fix(1:end-1,3 )-left_fix(1:end-1,4 ) left_fix(1:end-1,5 )-left_fix(1:end-1,7 ) left_fix(1:end-1,8 )-left_fix(1:end-1,9 ) left_fix(1:end-1,6 )]; ...
[up1_fix(1:end-1,1 )-up1_fix(1:end-1,2 ) up1_fix(1:end-1,3 )-up1_fix(1:end-1,4 ) up1_fix(1:end-1,5 )-up1_fix(1:end-1,7 ) up1_fix(1:end-1,8 )-up1_fix(1:end-1,9 ) up1_fix(1:end-1,6 )]; ...
[down_fix(1:end-1,1 )-down_fix(1:end-1,2 ) down_fix(1:end-1,3 )-down_fix(1:end-1,4 ) down_fix(1:end-1,5 )-down_fix(1:end-1,7 ) down_fix(1:end-1,8 )-down_fix(1:end-1,9 ) down_fix(1:end-1,6 )];];

EMG_test=[[forward_fix(2:end,1)-forward_fix(2:end,2 ) forward_fix(2:end,3 )-forward_fix(2:end,4 ) forward_fix(2:end,5 )-forward_fix(2:end,7 ) forward_fix(2:end,8 )-forward_fix(2:end,9 ) forward_fix(2:end,6 )]; ...
[back_fix(2:end,1)-back_fix(2:end,2 ) back_fix(2:end,3 )-back_fix(2:end,4 ) back_fix(2:end,5 )-back_fix(2:end,7 ) back_fix(2:end,8 )-back_fix(2:end,9 ) back_fix(2:end,6 )]; ...
[right_fix(2:end,1)-right_fix(2:end,2 ) right_fix(2:end,3 )-right_fix(2:end,4 ) right_fix(2:end,5 )-right_fix(2:end,7 ) right_fix(2:end,8 )-right_fix(2:end,9 ) right_fix(2:end,6 )]; ...
[left_fix(2:end,1 )-left_fix(2:end,2 ) left_fix(2:end,3 )-left_fix(2:end,4 ) left_fix(2:end,5 )-left_fix(2:end,7 ) left_fix(2:end,8 )-left_fix(2:end,9 ) left_fix(2:end,6 )]; ...
[up1_fix(2:end,1 )-up1_fix(2:end,2 ) up1_fix(2:end,3 )-up1_fix(2:end,4 ) up1_fix(2:end,5 )-up1_fix(2:end,7 ) up1_fix(2:end,8 )-up1_fix(2:end,9 ) up1_fix(2:end,6 )]; ...
[down_fix(2:end,1 )-down_fix(2:end,2 ) down_fix(2:end,3 )-down_fix(2:end,4 ) down_fix(2:end,5 )-down_fix(2:end,7 ) down_fix(2:end,8 )-down_fix(2:end,9 ) down_fix(2:end,6 )];];





x_train_x=[forward_cor(1:end-1,1); back_cor(1:end-1,1); right_cor(1:end-1,1); left_cor(1:end-1,1); up1_cor(1:end-1,1); down_cor(1:end-1,1);];
x_train_y=[forward_cor(1:end-1,2); back_cor(1:end-1,2); right_cor(1:end-1,2); left_cor(1:end-1,2); up1_cor(1:end-1,2); down_cor(1:end-1,2);];
x_train_z=[forward_cor(1:end-1,3); back_cor(1:end-1,3); right_cor(1:end-1,3); left_cor(1:end-1,3); up1_cor(1:end-1,3); down_cor(1:end-1,3);];

x_intent=[ones(length(forward_fix(1:end-1,7)),1);...
    2*ones(length(back_fix(1:end-1,7)),1);...
    3*ones(length(right_fix(1:end-1,7)),1);...
    4*ones(length(left_fix(1:end-1,7)),1);...
    5*ones(length(up1_fix(1:end-1,7)),1);...
    6*ones(length(down_fix(1:end-1,7)),1);];


X_train=[EMG_train x_train_x x_train_y x_train_z x_intent];

y_train_x=[forward_cor(2:end,1); back_cor(2:end,1); right_cor(2:end,1); left_cor(2:end,1); up1_cor(2:end,1); down_cor(2:end,1);];
y_train_y=[forward_cor(2:end,2); back_cor(2:end,2); right_cor(2:end,2); left_cor(2:end,2); up1_cor(2:end,2); down_cor(2:end,2);];
y_train_z=[forward_cor(2:end,3); back_cor(2:end,3); right_cor(2:end,3); left_cor(2:end,3); up1_cor(2:end,3); down_cor(2:end,3);];


% left right 手腕 z 相对肩膀，





%%

test_start=881; test_stop=1220; 



%%
X_test=X_train(test_start:test_stop,1:end-1);
train_start=881;train_stop=1220;
X_train=X_train(train_start:train_stop,:);
    

rng default
gprMdl_6 = fitrgp(X_train,y_train_x(train_start:train_stop,:),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood',...
    'OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_7 = fitrgp(X_train,y_train_y(train_start:train_stop,:),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood',...
    'OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_8 = fitrgp(X_train,y_train_z(train_start:train_stop,:),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood',...
    'OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus'));

gprMdl_1 = fitrgp(X_train,EMG_test(train_start:train_stop,1),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood',...
    'OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_2 = fitrgp(X_train,EMG_test(train_start:train_stop,2),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood',...
    'OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_3 = fitrgp(X_train,EMG_test(train_start:train_stop,3),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood',...
    'OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_4 = fitrgp(X_train,EMG_test(train_start:train_stop,4),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood',...
    'OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_5 = fitrgp(X_train,EMG_test(train_start:train_stop,5),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood',...
    'OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus'));




init_1=1/6;init_2=1/6;init_3=1/6;init_4=1/6;init_5=1/6;init_6=1/6;
lambda=1;
result=[]; result_y=[];result_y_upstd=[];result_y_downstd=[];







