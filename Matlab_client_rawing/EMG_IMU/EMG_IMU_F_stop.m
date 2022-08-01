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
kalmanF = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter

STEP = 2
STOP=0;

if STEP == 3
file='ylcoef_0119.mat';
coef_1=load(file).coef_1;
coef_2=load(file).coef_2;
fitinfo2=load(file).fitinfo2;
fitinfo4=load(file).fitinfo1;
coef_3=load(file).coef_3;
fitinfo3=load(file).fitinfo3;
Mdl_1=load(file).Mdl_1;
Mdl=load(file).Mdl;

end
% round1= load ('20220109yl5.mat').sychronize;  %
% round2= load ('20220109yl6.mat').sychronize;  %
% round3= load ('20220109yl4.mat').sychronize;
% round4= load ('20220109yl7.mat').sychronize;
% round5= load ('20220109yl7.mat').sychronize;
% round6= load ('20220109yl6.mat').sychronize;
% round7= load ('20220109yl7.mat').sychronize;
% round1= load ('20220113yl5_stop.mat').sychronize;  %
% round2= load ('20220113yl6_stop.mat').sychronize;  %
% round3= load ('20220113yl7_stop.mat').sychronize;
% round4= load ('20220113yl4.mat').sychronize;
% figure(77);plot(round1(86,:)); hold on; plot(round1(1,:))

round2= load ('20220120yl1_stop.mat').sychronize;  %
round4= load ('20220120yl2_stop.mat').sychronize;  %
round3= load ('20220120yl3_stop.mat').sychronize;  %
round1= load ('20220120yl4_stop.mat').sychronize;  %





train_X1=[]; train_Y1=[]; test_X1=[]; test_Y1=[];train_X5=[];test_Y4=[];
train_X2=[]; train_Y2=[]; test_X2=[]; test_Y2=[];test_X5=[];train_Y3=[];
train_Xo=[]; train_Yo=[]; test_Xo=[]; test_Yo=[];test_Y5=[];train_Y4=[];test_X6=[];
test_X3=[]; test_X4=[]; train_X3=[]; train_X4=[];test_Y3=[];train_Y5=[];train_X6=[];
start=1;

%%
for file = start:2
    sep=1;
    
    name_g=['round' num2str(file)];
    round_this=eval(name_g);
    round_this=round_this(:,1:end);

%     figure(1)
%     plot(round_this(7,:)); hold on;
%     
    
    
    % 归一化

        map_EMG=round_this(4:10,:);

    my_features=[map_EMG(1,:)-map_EMG(2,:); map_EMG(3,:)-map_EMG(4,:); map_EMG(5,:)-map_EMG(7,:);].';
    F_x=round_this(1,:).';
    F_z=round_this(3,:).';
    
    
    
    
%     max_F=max(F_x); min_F=min(F_x);
%     threhold=(max_F-min_F)/2.5;
%     intent1=find(F>max_F-threhold);
%     intent2=find(F<min_F+threhold);


    
    fea1=my_features(:,1).';
    fea2=my_features(:,2).';
    fea3=my_features(:,3).';
    fea4=map_EMG(6,:);
 %% ABY   

    
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
    
    
%     figure(1+cao)
%     plot(map_EMG)

all_y_filtered = kalmanv(all_y.');

caon1=all_y_filtered(1:end-1);
caon2=all_y_filtered(2:end);
nmb2=caon2-caon1;
% plot(all_y_filtered); hold on;
% plot(F_x)
%    all_y_filtered = kalmanv(all_y);  
    

  %%  
  
    sele_st=1; sele_end=7; bound1=700;bound2=1220;
    if file <= sep
            intent1=find(F_z>5);
             intent1=intent1(find(intent1<bound2));
            intent1=intent1(find(intent1>bound1));
            intent2=find(F_x<-5);
            intent2=intent2(find(intent2<bound1));
           
            intent3=find(F_x>5);

            intent3=intent3(find(intent3<bound1));
            
%             intent4_1=find(F_z == 0);intent4_2=find(F_x == 0);
%             intent4 = intersect(intent4_1,intent4_2);
%             intent4=intent4(find(intent4>2000));
%             
            if STEP == 2
            intent5_1=find(abs(F_z) <= 5);intent5_2=find(abs(F_x) <= 5);
            intent5 = intersect(intent5_1,intent5_2);
            intent5=intent5(find(intent5<bound2));
                    train_X5=[train_X5   [map_EMG(sele_st:sele_end,intent5); ]];
            end
            if STEP ~= 3
                intent6_1=find(F_z == 0);intent6_2=find(F_x == 0);
                intent6 = intersect(intent6_1,intent6_2);
                intent6=intent6(find(intent6>bound2));
                train_X6=[train_X6   [map_EMG(sele_st:sele_end,intent6); ]];
            end
            
            
            intent4=find(F_z<-5);
            
            intent4=intent4(find(intent4>bound1));           
            


            F_3=F_x(intent3);            
            F_1=F_z(intent1);
            F_2=F_x(intent2);
     

        train_X1=[train_X1  [map_EMG(sele_st:sele_end,intent1); ]];
        train_X2=[train_X2   [map_EMG(sele_st:sele_end,intent2); ]];
        train_X3=[train_X3   [map_EMG(sele_st:sele_end,intent3); ]];
        train_X4=[train_X4   [map_EMG(sele_st:sele_end,intent4); ]];

        
%         train_X1=[train_X1  map_EMG(sele_st:sele_end,intent1) ];
%         train_X2=[train_X2  map_EMG(sele_st:sele_end,intent2) ];
%         train_Xo=[train_Xo  map_EMG(sele_st:sele_end,intent_o) ];
        
%         train_X1=[train_X1  [fea1(:,intent1); fea2(:,intent1); fea3(:,intent1); fea4(:,intent1);] ];
%         train_X2=[train_X2  [fea1(:,intent2); fea2(:,intent2); fea3(:,intent2); fea4(:,intent2);] ];
%         train_Xo=[train_Xo  [fea1(:,intent_o); fea2(:,intent_o); fea3(:,intent_o); fea4(:,intent_o);] ];

        train_Y3=[train_Y3  ;F_3];        
        train_Y1=[train_Y1  ;F_1];
        train_Y2=[train_Y2  ;F_2];
     
    else
        
            intent1=find(F_z>5);
            
            intent1=intent1(find(intent1<bound2));
            intent1=intent1(find(intent1>bound1));
            
            
            intent2=find(F_x<-5);
            intent2=intent2(find(intent2<bound1));

            intent3=find(F_x>5);
            
            intent3=intent3(find(intent3<bound1));
            
%             intent4_1=find(F_z == 0);intent4_2=find(F_x == 0);
%             intent4 = intersect(intent4_1,intent4_2);
%             intent4=intent4(find(intent4>2000));
            
            if STEP == 2
            intent5_1=find(abs(F_z) <= 5);intent5_2=find(abs(F_x) <= 5);
            intent5 = intersect(intent5_1,intent5_2);
            intent5=intent5(find(intent5<bound2));
            test_X5=[test_X5   [map_EMG(sele_st:sele_end,intent5); ]];
            end
            if STEP ~= 3
                intent6_1=find(F_z == 0);intent6_2=find(F_x == 0);
                intent6 = intersect(intent6_1,intent6_2);
                intent6=intent6(find(intent6>bound2));
                test_X6=[test_X6   [map_EMG(sele_st:sele_end,intent6); ]];
            end

            intent4=find(F_z<-5);
            intent4=intent4(find(intent4>bound1));
            
            

            F_3=F_x(intent3);            
            F_1=F_z(intent1);
            F_2=F_x(intent2);

        
        test_X1=[test_X1  [map_EMG(sele_st:sele_end,intent1);]];
        test_X2=[test_X2  [map_EMG(sele_st:sele_end,intent2); ]];
        test_X3=[test_X3   [map_EMG(sele_st:sele_end,intent3); ]];
        test_X4=[test_X4   [map_EMG(sele_st:sele_end,intent4); ]];
        
%         test_X1=[test_X1  [fea1(:,intent1); fea2(:,intent1); fea3(:,intent1); fea4(:,intent1);] ];
%         test_X2=[test_X2  [fea1(:,intent2); fea2(:,intent2); fea3(:,intent2); fea4(:,intent2);] ];
%         test_Xo=[test_Xo  [fea1(:,intent_o); fea2(:,intent_o); fea3(:,intent_o); fea4(:,intent_o);] ];  
        
        
        test_Y3=[test_Y3  ;F_3];
   
        test_Y1=[test_Y1;  F_1];
        test_Y2=[test_Y2 ; F_2];

 
last_F_filtered2=0;output=[];predict_F=[];
% % %% predict in loop
if STEP == 3
        predict_test_y=[];predict_test_y_KKK=[]; 
        for inside =1:size(map_EMG,2)
  inside
        this_frame_EMG=[map_EMG(2:end,inside)].'; 
        
        outclass_judge = predict(Mdl_1,map_EMG(1,inside));
        
    if outclass_judge == 1
       
           outclass = predict(Mdl,this_frame_EMG);
           if outclass == 1
                output=[output 1];

%                  pre_y=0;
           elseif outclass == 2
                output=[output 2];
%                 pre_y=this_frame_EMG*coef_2+fitinfo2.Intercept;  
            elseif outclass == 3
                output=[output 3];    
%                 pre_y=this_frame_EMG*coef_3+fitinfo3.Intercept;
            elseif outclass == 4
                output=[output 4];    
%                 pre_y=0;
            else
                output=[output 0]; 
%                 pre_y=0;
           end
            if length(output) >=11
                how3=length(find(output(end-10:end)==3));
                how2=length(find(output(end-10:end)==2));
                if how3>how2
                    pre_y=(this_frame_EMG*coef_3+fitinfo3.Intercept)*how3/10;
                else
                    pre_y=(this_frame_EMG*coef_2+fitinfo2.Intercept)*how2/10;
                end
            else
                pre_y=0;
            end
           
           
           
            predict_F=[predict_F pre_y];
             F_filtered2 = kalmanF(pre_y);
             predict_test_y_KKK=[predict_test_y_KKK; F_filtered2;];
            F_filtered2=0;

             
    else
        output=[output -1]; 
        STOP=STOP+1;
        disp('EMG nonooo')
    end
             
        end
    figure(45+file);
    plot(output); hold on;
    plot(predict_F); hold on;
    plot(predict_test_y_KKK); hold on; plot(F_x); hold on; plot(F_z)
    legend('分类','预测','滤波','真实x力','真实z力')
end
    end

      
% figure(1+file)
% plot(all_y); hold on;
% plot(all_b); hold on;
% plot(all_a)
% legend('绕x轴转的角度','绕y轴转的角度','绕z轴转的角度')
% 
%   %% LASSO
% rng default % For reproducibility 
% 
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
if STEP == 2
training=[train_X1 train_X2 train_X3 train_X4 train_X5].';
group=[1*ones(size(train_X1,2),1); 2*ones(size(train_X2,2),1); 3*ones(size(train_X3,2),1); 4*ones(size(train_X4,2),1); 5*ones(size(train_X5,2),1);];
sample=[test_X1 test_X2 test_X3 test_X4 test_X5].';
test_answer=[1*ones(size(test_X1,2),1); 2*ones(size(test_X2,2),1); 3*ones(size(test_X3,2),1); 4*ones(size(test_X4,2),1); 5*ones(size(test_X5,2),1);];

% sample=training;
% test_answer=group;
end
% % training=[train_X1 train_X2 train_X3].';
% group=[1*ones(size(train_X1,2),1); 2*ones(size(train_X2,2),1); 3*ones(size(train_X3,2),1);];
% sample=[test_X1 test_X2 test_X3].';
% test_answer=[1*ones(size(test_X1,2),1); 2*ones(size(test_X2,2),1); 3*ones(size(test_X3,2),1); ];
% [outclass, SigmaHat, logDetSigma,gmeans, posterior] = QDA_classify(sample, training, group);
% training=[train_X1 train_X2 train_X3 train_X4].';
% group=[1*ones(size(train_X1,2),1); 2*ones(size(train_X2,2),1); 3*ones(size(train_X3,2),1); 4*ones(size(train_X4,2),1);];
% sample=[test_X1 test_X2 test_X3 test_X4].';
% test_answer=[1*ones(size(test_X1,2),1); 2*ones(size(test_X2,2),1); 3*ones(size(test_X3,2),1); 4*ones(size(test_X4,2),1); ];

if STEP == 1

training=[train_X1 train_X2 train_X3 train_X4 train_X6].';
group=[1*ones(size(train_X1,2),1); 1*ones(size(train_X2,2),1); 1*ones(size(train_X3,2),1); 1*ones(size(train_X4,2),1); 2*ones(size(train_X6,2),1);];
sample=[test_X1 test_X2 test_X3 test_X4 test_X6].';
test_answer=[1*ones(size(test_X1,2),1); 1*ones(size(test_X2,2),1); 1*ones(size(test_X3,2),1); 1*ones(size(test_X4,2),1); 2*ones(size(test_X6,2),1);];

training=training(:,1);
sample=sample(:,1);

% % SVM
X=training;
Y=group;
classOrder = unique(Y);
% t = templateSVM('Standardize',true);
t = templateSVM('KernelFunction','gaussian');
PMdl = fitcecoc(X,Y,'Holdout',0.010,'Learners',t,'ClassNames',classOrder);
Mdl = PMdl.Trained{1};           % Extract trained, compact classifier

% sample=training;
% test_answer=group;

outclass = predict(Mdl,sample);


delta=outclass-test_answer;
correct=find(delta==0);
answer=length(correct)/size(sample,1)

figure(99)
plot(outclass); hold on; plot(test_answer)
% figure;plot(training)
% figure;plot(sample)
end


% % % KNN
% mdl = fitcknn(training, group,'OptimizeHyperparameters','auto',...
%     'HyperparameterOptimizationOptions',...
%     struct('AcquisitionFunctionName','expected-improvement-plus'),'HyperparameterOptimizationOptions', struct('UseParallel',true)) 
% outclass = predict(mdl,sample);

if STEP == 2
training=training(:,2:end);
sample=sample(:,2:end);
% % SVM
X=training;
Y=group;
classOrder = unique(Y);
% t = templateSVM('Standardize',true);
t = templateSVM('KernelFunction','gaussian');
PMdl = fitcecoc(X,Y,'Holdout',0.010,'Learners',t,'ClassNames',classOrder);
Mdl = PMdl.Trained{1};           % Extract trained, compact classifier

% sample=training;
% test_answer=group;

outclass = predict(Mdl,sample);


delta=outclass-test_answer;
correct=find(delta==0);
answer=length(correct)/size(sample,1)

figure(99)
plot(outclass); hold on; plot(test_answer)
end
%% LASSO
if STEP == 2
    
train_X22=train_X2(2:end,:);
train_X44=train_X4(2:end,:);
test_X22=test_X2(2:end,:);
test_X44=test_X1(2:end,:);
train_X33=train_X3(2:end,:);
test_X33=test_X3(2:end,:);

[coef_4,fitinfo4] = lasso(train_X44.',train_Y4,'Weights',ones(size(train_X44.',1),1),'Alpha',0.75,'Lambda',0.005);
figure(15)
pre_y4=test_X44.'*coef_4+fitinfo4.Intercept;


F_filtered4 = kalmanz(pre_y4);
plot(pre_y4); hold on; plot(test_Y4); hold on; plot(F_filtered4)
legend('pre','rea','kkk')



[coef_2,fitinfo2] = lasso(train_X22.',train_Y2,'Weights',ones(size(train_X22.',1),1),'Alpha',0.75,'Lambda',0.005);
figure(16)
pre_y2=test_X22.'*coef_2+fitinfo2.Intercept;

F_filtered2 = kalmanz(pre_y2);
plot(pre_y2); hold on; plot(test_Y2); hold on; plot(F_filtered2)
legend('pre','rea','kkk')


[coef_3,fitinfo3] = lasso(train_X33.',train_Y3,'Weights',ones(size(train_X33.',1),1),'Alpha',0.75,'Lambda',0.005);
figure(17)
pre_y3=test_X33.'*coef_3+fitinfo3.Intercept;

F_filtered3 = kalmanz(pre_y3);
plot(pre_y3); hold on; plot(test_Y3); hold on; plot(F_filtered3)
legend('pre','rea','kkk')



training=[train_X1 train_X2 train_X3 train_X4 train_X6].';
group=[1*ones(size(train_X1,2),1); 1*ones(size(train_X2,2),1); 1*ones(size(train_X3,2),1); 1*ones(size(train_X4,2),1); 2*ones(size(train_X6,2),1);];
sample=[test_X1 test_X2 test_X3 test_X4 test_X6].';
test_answer=[1*ones(size(test_X1,2),1); 1*ones(size(test_X2,2),1); 1*ones(size(test_X3,2),1); 1*ones(size(test_X4,2),1); 2*ones(size(test_X6,2),1);];

training=training(:,1);
sample=sample(:,1);
figure;plot(training)
figure;plot(sample)
% % SVM
X=training;
Y=group;
classOrder = unique(Y);
% t = templateSVM('Standardize',true);
t = templateSVM('KernelFunction','gaussian');
PMdl = fitcecoc(X,Y,'Holdout',0.010,'Learners',t,'ClassNames',classOrder);
Mdl_1 = PMdl.Trained{1};           % Extract trained, compact classifier
outclass = predict(Mdl_1,sample);


delta=outclass-test_answer;
correct=find(delta==0);
answer=length(correct)/size(sample,1)

save('ylcoef_0119.mat','coef_2','coef_1','fitinfo2','fitinfo1','coef_3','fitinfo3','Mdl','Mdl_1')

end
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


% out = auto_save(1)
% str = strcat('data-',out, ...
% 'LY', '.mat');
% save(str)

% clear all
% load data-15-Jan-2022112031LY.mat
% nargin1


figure(1)
plot(round1(4,:)); hold on; plot(round2(4,:))
figure(2)
plot(round1(5,:)); hold on; plot(round2(5,:))
figure(3)
plot(round1(6,:)); hold on; plot(round2(6,:))
figure(4)
plot(round1(7,:)); hold on; plot(round2(7,:))
figure(5)
plot(round1(8,:)); hold on; plot(round2(8,:))
figure(6)
plot(round1(9,:)); hold on; plot(round2(9,:))
figure(7)
plot(round1(10,:)); hold on; plot(round2(10,:))
