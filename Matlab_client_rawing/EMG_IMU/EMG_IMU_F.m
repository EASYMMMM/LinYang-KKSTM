close all;clear;clc;
%%
R=0.005;
kalmanz = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
kalmanv = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter

coef_1=load('LASSO_coef.mat').coef_1;
coef_2=load('LASSO_coef.mat').coef_2;
fitinfo2=load('LASSO_coef.mat').fitinfo2;
fitinfo1=load('LASSO_coef.mat').fitinfo1;
Mdl=load('LASSO_coef.mat').Mdl;

round1= load ('20220109yl5.mat').sychronize;  %
round2= load ('20220109yl6.mat').sychronize;  %
round3= load ('20220109yl4.mat').sychronize;
round4= load ('20220109yl7.mat').sychronize;
% round5= load ('20220109yl7.mat').sychronize;
% round6= load ('20220109yl6.mat').sychronize;
% round7= load ('20220109yl7.mat').sychronize;
% round1= load ('20220113yl5_stop.mat').sychronize;  %
% round2= load ('20220113yl6_stop.mat').sychronize;  %
% round3= load ('20220113yl7_stop.mat').sychronize;
% round4= load ('20220113yl4.mat').sychronize;



train_X1=[]; train_Y1=[]; test_X1=[]; test_Y1=[];
train_X2=[]; train_Y2=[]; test_X2=[]; test_Y2=[];
train_Xo=[]; train_Yo=[]; test_Xo=[]; test_Yo=[];

start=1;

%%
for file = start:4
    sep=1;
    
    name_g=['round' num2str(file)];
    round_this=eval(name_g);
    round_this=round_this(:,1:end);

    % 归一化
    if file == start
         [map_EMG, ps]=mapminmax(round_this(4:10,:),0,1);    
    else
         map_EMG=mapminmax('apply',round_this(4:10,:),ps);
    end
%     map_EMG=round_this(4:10,:);
%     [map_EMG,PS]=mapminmax(round_this(4:10,:));
    
    
    
    my_features=[map_EMG(1,:)-map_EMG(2,:); map_EMG(3,:)-map_EMG(4,:); map_EMG(5,:)-map_EMG(7,:);].';
    F_x=round_this(1,:).';
    F_z=round_this(3,:).';
    
    
    
    
    max_F=max(F_x); min_F=min(F_x); 
%     threhold=(max_F-min_F)/2.5;
%     intent1=find(F>max_F-threhold);
%     intent2=find(F<min_F+threhold);


    
    fea1=my_features(:,1).';
    fea2=my_features(:,2).';
    fea3=my_features(:,3).';
    fea4=map_EMG(6,:);
 %% ABY   
    jerk=[];
    for nmb = 1:length(F_x)-1
        this=F_x(nmb);
        next=F_x(nmb+1);
        jerk=[jerk next-this];
    end
    jerk=[jerk next-this];
    
    
    data1=round_this(11:35,:).';
    data2=round_this(36:60,:).';
    data3=round_this(61:85,:).';

    filter_A=[];all_a=[]; all_b=[]; all_y=[];
    for leno = 1:size(data1,1)
        this1 = data1(leno, :);
        this2 = data2(leno, :);
        this3 = data3(leno, :);
        R1 = rotate_matrix(this1);
        R2 = rotate_matrix(this2);
        R3 = rotate_matrix(this3);
        R31 = R1.'*R3;
        R23 = R3.'*R2;
        [b,a,y] = inverse_angle(R31);
        all_a=[all_a a];
        all_b=[all_b b];
        all_y=[all_y y];
    end    
%% 处理角度
    all_y=all_y-all_y(1);
%     after_all_y=all_y(1);
    for each_y = 1:length(all_y)-1
        this_y = all_y(each_y);
        next_y = all_y(each_y+1);
        if next_y<-320
            next_y=next_y+360;
            all_y(each_y+1) = next_y;
        end
        if abs(next_y-this_y)>10
            all_y(each_y+1) = this_y;
%             after_all_y=[after_all_y next_y];
        end
    end
    figure(1+file)
    plot(all_y); hold on;

all_y_filtered = kalmanv(all_y.');

%    all_y_filtered = kalmanv(all_y);  
    caon1=all_y_filtered(1:end-1);
caon2=all_y_filtered(2:end);
nmb2=caon2-caon1;
plot(all_y_filtered); hold on;
plot(F_x); hold on;
plot(nmb2)

% plot(all_y_filtered); hol

  %%  
  
    sele_st=1; sele_end=7;
    if file <= sep
            intent1=find(F_x>20);
            intent2=find(F_x<-20);
            intent_o=1:length(F_x); intent_o([intent1; intent2])=[];
            F_1=F_x(intent1);
            F_2=F_x(intent2);
            F_other=F_x(intent_o);
        v1 = seev(all_y_filtered, intent1);
        v2 = seev(all_y_filtered, intent2);
        v3 = seev(all_y_filtered, intent_o);
        train_X1=[train_X1  [map_EMG(sele_st:sele_end,intent1); v1]];
        train_X2=[train_X2   [map_EMG(sele_st:sele_end,intent2); v2]];
        train_Xo=[train_Xo   [map_EMG(sele_st:sele_end,intent_o); v3]];
        
%         train_X1=[train_X1  map_EMG(sele_st:sele_end,intent1) ];
%         train_X2=[train_X2  map_EMG(sele_st:sele_end,intent2) ];
%         train_Xo=[train_Xo  map_EMG(sele_st:sele_end,intent_o) ];
        
%         train_X1=[train_X1  [fea1(:,intent1); fea2(:,intent1); fea3(:,intent1); fea4(:,intent1);] ];
%         train_X2=[train_X2  [fea1(:,intent2); fea2(:,intent2); fea3(:,intent2); fea4(:,intent2);] ];
%         train_Xo=[train_Xo  [fea1(:,intent_o); fea2(:,intent_o); fea3(:,intent_o); fea4(:,intent_o);] ];
        
        train_Y1=[train_Y1  ;F_1];
        train_Y2=[train_Y2  ;F_2];
        train_Yo=[train_Yo  ;F_other];        
    else
        
            intent1=find(F_x>0);
            intent2=find(F_x<0);
            intent_o=1:length(F_x); intent_o([intent1; intent2])=[];
            F_1=F_x(intent1);
            F_2=F_x(intent2);
            F_other=F_x(intent_o); 
         v1 = seev(all_y_filtered, intent1);
        v2 = seev(all_y_filtered, intent2);
        v3 = seev(all_y_filtered, intent_o);           
        test_X1=[test_X1  [map_EMG(sele_st:sele_end,intent1); v1]];
        test_X2=[test_X2  [map_EMG(sele_st:sele_end,intent2); v2]];
        test_Xo=[test_Xo  [map_EMG(sele_st:sele_end,intent_o); v3]];
        
%         test_X1=[test_X1  [fea1(:,intent1); fea2(:,intent1); fea3(:,intent1); fea4(:,intent1);] ];
%         test_X2=[test_X2  [fea1(:,intent2); fea2(:,intent2); fea3(:,intent2); fea4(:,intent2);] ];
%         test_Xo=[test_Xo  [fea1(:,intent_o); fea2(:,intent_o); fea3(:,intent_o); fea4(:,intent_o);] ];  
        

        
        test_Y1=[test_Y1;  F_1];
        test_Y2=[test_Y2 ; F_2];
        test_Yo=[test_Yo ; F_other];       
 

% %% predict in loop
%         predict_test_y=[];predict_test_y_KKK=[];
%         for inside = 1:size(map_EMG,2)
%                 this_frame=map_EMG(:,inside);
%                 outclass = predict(Mdl,this_frame.');
%                 pre_intent1=find(outclass == 1);
%                 pre_intent2=find(outclass == 2);
%                 pre_X1=this_frame(:,pre_intent1);
%                 pre_X2=this_frame(:,pre_intent2);
%                 pre_y1=pre_X1.'*coef_1+fitinfo1.Intercept;
%                 pre_y2=pre_X2.'*coef_2+fitinfo2.Intercept;
%                 F_filtered2 = kalmanz([pre_y2 pre_y1]);
%                 predict_test_y=[predict_test_y; pre_y1; pre_y2;];
%                 predict_test_y_KKK=[predict_test_y_KKK; F_filtered2;];
%         end
%     figure(45+file)
%     plot(predict_test_y); hold on; 
%     plot(predict_test_y_KKK); hold on; 
%     plot(F_x);
%     legend('pre','KkK','real')

    end

              figure(66+file);
        plot(v2); hold on; plot(F_2/20);
% figure(1+file)
% plot(all_y); hold on;
% plot(all_b); hold on;
% plot(all_a)
% legend('绕x轴转的角度','绕y轴转的角度','绕z轴转的角度')

%   %% LASSO
% rng default % For reproducibility 

% my_features=[my_features all_a.'];
% log_fea=log(my_features+2)
% 
% [coef_1,fitinfo] = lasso(map_EMG.',F,'Weights',ones(size(my_features,1),1),'Alpha',0.75,'Lambda',0.005);
% figure(15+file)
% pre_y=map_EMG.'*coef_1+fitinfo.Intercept;
%  
% F_filtered1 = kalmanz(pre_y);
% plot(pre_y); hold on; plot(F); hold on; plot(F_filtered1)
% legend('pre','rea','kkk')

%% 俩俩看
%     figure(25+file); 
%     plot(fea1); hold on; 
%     plot(fea2); hold on; 
%     plot(fea3); hold on; 
% %     plot(all_y-140); hold on; 
%     plot(F/20); 
% %     plot(jerk); 
%     legend('1-2','3-4','5-7','F')
  %% 一个看  
%    figure(35+file); 
%    plot(map_EMG(5,:)); hold on;
%    plot(map_EMG(6,:)); hold on;
%    plot(map_EMG(7,:)); 
%    plot(F/20); 
%     legend('5','6','7')


end

%QDA

% training=[train_X1 train_X2 train_Xo].';
% group=[1*ones(size(train_X1,2),1); 2*ones(size(train_X2,2),1); 3*ones(size(train_Xo,2),1);];
% sample=[test_X1 test_X2 test_Xo].';
% test_answer=[1*ones(size(test_X1,2),1); 2*ones(size(test_X2,2),1); 3*ones(size(test_Xo,2),1);];
training=[train_X1 train_X2].';
group=[1*ones(size(train_X1,2),1); 2*ones(size(train_X2,2),1);];
sample=[test_X1 test_X2].';
test_answer=[1*ones(size(test_X1,2),1); 2*ones(size(test_X2,2),1);];

% [outclass, SigmaHat, logDetSigma,gmeans, posterior] = QDA_classify(sample, training, group);


% % % % KNN
% mdl = fitcknn(training, group,'OptimizeHyperparameters','auto',...
%     'HyperparameterOptimizationOptions',...
%     struct('AcquisitionFunctionName','expected-improvement-plus'),'HyperparameterOptimizationOptions', struct('UseParallel',true)) 
% outclass = predict(mdl,sample);



% SVM
X=training;
Y=group;
classOrder = unique(Y);
% t = templateSVM('Standardize',true);
t = templateSVM('KernelFunction','gaussian');
PMdl = fitcecoc(X,Y,'Holdout',0.010,'Learners',t,'ClassNames',classOrder);
Mdl = PMdl.Trained{1};           % Extract trained, compact classifier
outclass = predict(Mdl,sample);


delta=outclass-test_answer;
correct=find(delta==0);
answer=length(correct)/size(sample,1)

%% LASSO


% [coef_1,fitinfo1] = lasso(train_X1.',train_Y1,'Weights',ones(size(train_X1.',1),1),'Alpha',0.75,'Lambda',0.005);
% figure(15)
% pre_y1=test_X1.'*coef_1+fitinfo1.Intercept;
% 
% 
% F_filtered1 = kalmanz(pre_y1);
% plot(pre_y1); hold on; plot(test_Y1); hold on; plot(F_filtered1)
% legend('pre','rea','kkk')
% 
% 
% 
% [coef_2,fitinfo2] = lasso(train_X2.',train_Y2,'Weights',ones(size(train_X2.',1),1),'Alpha',0.75,'Lambda',0.005);
% figure(16)
% pre_y2=test_X2.'*coef_2+fitinfo2.Intercept;
% 
% 
% F_filtered2 = kalmanz(pre_y2);
% plot(pre_y2); hold on; plot(test_Y2); hold on; plot(F_filtered2)
% legend('pre','rea','kkk')
% 
% save('LASSO_coef.mat','coef_2','coef_1','fitinfo2','fitinfo1','Mdl')
%% sigmoid

%     x=training(:,5);
%     y=training(:,6);
%     z=[train_Y1; train_Y2];
% %     g = fittype( @(a1,b1,c1,d1,x) ...
% %         a1./(1+exp(-c1*x+d1))+b1, ...
% %              'independent', {'x'}, ...
% %             'dependent', 'z' ); 
%     g = fittype( @(a1,b1,c1,d1,a2,c2,d2,x,y) ...
%         a1./(1+exp(-c1*x+d1))+b1+a2./(1+exp(-c2*y+d2)), ...
%              'independent', {'x', 'y'}, ...
%             'dependent', 'z' ); 
%     opts= fitoptions('Method', 'NonlinearLeastSquares');
%     cfun=fit([x y],z,g,opts);
%     yi=cfun([x y]);
%     figure(66+file)
%     plot(z); hold on; plot(yi)
%     legend('real value', 'fit value')