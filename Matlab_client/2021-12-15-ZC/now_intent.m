close all;clear;clc;
warning('off');
% this file is used to divide the raw series of data into labeled data.
% 2021-12-07

% % 左右上下后前——123456
%%
kalmanz = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter

round1= load ('tableup1_20211215_zengcheng.mat').round1;  % good 
round3= load ('tableup2_20211215_zengcheng.mat').sychronize;  % good
round5= load ('tableup3_20211215_zengcheng.mat').sychronize; % fine 
round7= load ('tableup4_20211215_zengcheng.mat').sychronize;
round2= load ('tableback1_20211215_zengcheng.mat').sychronize; % good
round4= load ('tableback2_20211215_zengcheng.mat').sychronize;  % fine
round6= load ('tableback4_20211215_zengcheng.mat').sychronize;
round8= load ('tableback5_20211215_zengcheng.mat').sychronize;


% 左右上下后前——123456
all_correct_rate=[];window=10
count=1;
list_seperate_rate=[3 5 7];
ALL_MSE=[];count_fuck=1;ALL_result=[];ALL_result_use=[];ALL_corr=[];
for seperate_rate = list_seperate_rate
    
    seperate_rate
%%
all_round_train_X_EMG=[]; all_round_train_Y=[];all_round_test_Y=[];all_round_test_X_EMG=[];
for file = 1:8
%     if file == 2
%         continue
%     end    
    
    
    each_file=[];
    all_round_thisd_train_X_EMG=[]; all_round_thisd_train_Y=[];all_round_thisd_test_X_EMG=[]; all_round_thisd_test_Y=[];
    for FLAG = 1:3
        if FLAG == 2 % y
            wrist_direction=8; lower_bound=40; upper_bound=900; label1=3; label2=4; howmany = 800;
        elseif FLAG == 3 % z
            wrist_direction=9; lower_bound=60; upper_bound=500; label1=5; label2=6; howmany = 800;
        elseif FLAG == 1  % x
            wrist_direction=7; lower_bound=85; upper_bound=750; label1=1; label2=2; howmany = 800;
        end



    
    
    % 不必要的
    if mod(file,2) == 1 && FLAG == 1
        continue
    end
    
    if mod(file,2) == 0 && FLAG == 2
        continue
    end

    
    
    
    
    name_g=['round' num2str(file)];
    round_this=eval(name_g);
    round_this = transform(round_this) ; % 以人为参考系
       

    t=1:length(round_this(:,wrist_direction));
    p=polyfit(t,round_this(:,wrist_direction),20);
    yi=polyval(p,t);

    
    slope=[];
    for i = 1:length(yi)-1
        this = yi(i);
        next=yi(i+1);
        delta=next-this;
        slope=[slope delta];
    end


    sorted_slope=sort(abs(slope));
    threhold=sorted_slope(howmany);
    if FLAG == 3
        which_want=find(slope > threhold);
    else
        which_want=find(abs(slope) > threhold);
    end
    
    new_which_want=[];
    for z = 1:length(which_want)
        oo=which_want(z);
        
        if file ~= 2
        
            if oo < upper_bound && oo > lower_bound
                new_which_want=[new_which_want oo];
            end
        else
            if FLAG == 3
                if oo < 423 && oo > 229
                    new_which_want=[new_which_want oo];
                end
            else
                if (oo < upper_bound && oo > 420) || (oo < 228 && oo > lower_bound)
                    new_which_want=[new_which_want oo];
                end
            end
                
        end
        
    end

    %%  处理连续问题
    
    
    
    want_axis=yi(new_which_want);
    
    
    
    
    
    
    each_file=[each_file new_which_want];
    
    others_nolabel=[1:size(round_this,1)];
    others_nolabel(new_which_want)=[];

%      %——注释掉，调试用的,xyz
%     figure(4+file);
%     plot(t,round_this(:,wrist_direction),':o',t,yi,'-*'); hold on;
%     plot(new_which_want,want_axis,'o','MarkerSize',10)
    


%% 滑动窗
result_window=[]; half=5;
for roww = 1:size(round_this,1)-10
    this_window=(sum(round_this(roww+half+1:roww+10,:),1)+sum(round_this(roww+1:roww+half,:),1))/half*2;
    result_window=[result_window;this_window;];
end
round_this=result_window;

    %% 归一化吗
    round_this_EMG_map=mapminmax(round_this(:,13:21).',0,1).';
%     round_this_EMG_map=round_this(:,13:21);
    
    new_this_round=[round_this(:,1:12) round_this_EMG_map round_this(:,22:end)];
    this_round_train_X=[];this_round_train_Y=[];this_round_test_X=[];this_round_test_Y=[];
    this_round_train_EMG=[];this_round_test_EMG=[];
    this_round_updown_xyz_Y=[];this_round_EMG_Y=[]; this_round_pass0_Y=[];
    for each_slop = 1:length(new_which_want)
        each_slope = new_which_want(each_slop); % 这些是第几个留下来的点
        this_slope = slope(each_slope); % 这些是这几个点的斜率

        this_round_EMG_each_X= [[new_this_round(each_slope,1+12)-new_this_round(each_slope,2+12)]...
            [new_this_round(each_slope,3+12)-new_this_round(each_slope,4+12)]...
            [new_this_round(each_slope,5+12)-new_this_round(each_slope,7+12)]...
            [new_this_round(each_slope,8+12)-new_this_round(each_slope,9+12)]...
            [new_this_round(each_slope,6+12)]];
  
         this_round_pass0_each_X= [[new_this_round(each_slope,1+21)-new_this_round(each_slope,2+21)]...
            [new_this_round(each_slope,3+21)-new_this_round(each_slope,4+21)]...
            [new_this_round(each_slope,5+21)-new_this_round(each_slope,7+21)]...
            [new_this_round(each_slope,8+21)-new_this_round(each_slope,9+21)]...
            [new_this_round(each_slope,6+21)]];
  
        
        
    t=1:length(round_this(:,wrist_direction));
    px=polyfit(t,round_this(:,7),20);
    py=polyfit(t,round_this(:,8),20);
    pz=polyfit(t,round_this(:,9),20);
    yz=polyval(pz,t);
       yx=polyval(px,t);
       yy=polyval(py,t);
       
        if this_slope > 0  % 训练的部分有拟合
            this_round_train_X=[this_round_train_X; [yx(each_slope) yy(each_slope) yz(each_slope)  label1];];
            this_round_train_EMG=[this_round_train_EMG; [yx(each_slope) yy(each_slope) yz(each_slope) round_this_EMG_map(each_slope,:)  label1];];
%           if  each_slop <= window
%               this_round_train_EMG_slope= [];
%           else
%             this_round_train_EMG_slope=[this_round_train_EMG_slope; [yx(each_slope)-yx(each_slope-window) yy(each_slope)-yy(each_slope-window) yz(each_slope)-yz(each_slope-window) round_this_EMG_map(each_slope,:)  label1];];
%           end
            
        else
            this_round_train_X=[this_round_train_X; [yx(each_slope) yy(each_slope) yz(each_slope) label2];];
            this_round_train_EMG=[this_round_train_EMG; [yx(each_slope) yy(each_slope) yz(each_slope) round_this_EMG_map(each_slope,:)  label2];];
%            if  each_slop <= window
%               this_round_train_EMG_slope= [];
%           else
%             this_round_train_EMG_slope=[this_round_train_EMG_slope; [yx(each_slope)-yx(each_slope-window) yy(each_slope)-yy(each_slope-window) yz(each_slope)-yz(each_slope-window) round_this_EMG_map(each_slope,:)  label2];];
%            end
%           
        end
        

        
        
        if this_slope > 0  % 测试的部分没有拟合
            this_round_test_X=[this_round_test_X; [new_this_round(each_slope,7:9) label1];];
            this_round_test_EMG=[this_round_test_EMG; [new_this_round(each_slope,7:9) round_this_EMG_map(each_slope,:)  label1];];
%           if  each_slop <= window
%               this_round_test_EMG_slope= [];
%           else
%             this_round_test_EMG_slope=[this_round_test_EMG_slope; [new_this_round(each_slope,7)-new_this_round(each_slope-window,7) new_this_round(each_slope,8)-new_this_round(each_slope-window,8) new_this_round(each_slope,9)-new_this_round(each_slope-window,9) round_this_EMG_map(each_slope,:)  label1];];
%           end
        
        else
            this_round_test_X=[this_round_test_X; [new_this_round(each_slope,7:9) label2];];
            this_round_test_EMG=[this_round_test_EMG; [new_this_round(each_slope,7:9) round_this_EMG_map(each_slope,:)  label2];];
%           if  each_slop <= window
%               this_round_test_EMG_slope= [];
%           else
%             this_round_test_EMG_slope=[this_round_test_EMG_slope; [new_this_round(each_slope,7)-new_this_round(each_slope-window,7) new_this_round(each_slope,8)-new_this_round(each_slope-window,8) new_this_round(each_slope,9)-new_this_round(each_slope-window,9) round_this_EMG_map(each_slope,:)  label2];];
%           end
        end
        
        this_round_updown_xyz_Y=[this_round_updown_xyz_Y; new_this_round(each_slope+1,7:9);];
        
        this_round_EMG_each_Y= [[new_this_round(each_slope+1,1+12)-new_this_round(each_slope+1,2+12)]...
            [new_this_round(each_slope+1,3+12)-new_this_round(each_slope+1,4+12)]...
            [new_this_round(each_slope+1,5+12)-new_this_round(each_slope+1,7+12)]...
            [new_this_round(each_slope+1,8+12)-new_this_round(each_slope+1,9+12)]...
            [new_this_round(each_slope+1,6+12)]];
        this_round_EMG_Y=[this_round_EMG_Y; this_round_EMG_each_Y;];  % 
        
         this_round_pass0_each_Y= [[new_this_round(each_slope+1,1+21)-new_this_round(each_slope+1,2+21)]...
            [new_this_round(each_slope+1,3+21)-new_this_round(each_slope+1,4+21)]...
            [new_this_round(each_slope+1,5+21)-new_this_round(each_slope+1,7+21)]...
            [new_this_round(each_slope+1,8+21)-new_this_round(each_slope+1,9+21)]...
            [new_this_round(each_slope+1,6+21)]];
        this_round_pass0_Y=[this_round_pass0_Y; this_round_pass0_each_Y;];
        
        this_round_test_Y=[this_round_test_Y; new_this_round(each_slope+1,7:9);];
        

        
        
    end
    
    if FLAG == 1 
        see_who=[5 7];
    elseif FLAG == 2
        see_who=[3 4];

    else
        see_who=[8 9];
    end
    
    


this_round_train_EMG=[this_round_train_EMG(:,1:end-1) this_round_EMG_Y this_round_train_EMG(:,end)];
this_round_test_EMG=[this_round_test_EMG(:,1:end-1) this_round_EMG_Y this_round_test_EMG(:,end)];

     %——注释掉，调试用的,EMG
    figure(count);
    count=count+1;
    plot(round_this_EMG_map(:,see_who));  
    hold on;
    plot(new_which_want,this_round_train_EMG(:,see_who+3));  
    hold on;
    plot(others_nolabel,round_this_EMG_map(others_nolabel,see_who),'ro');   
    legend('ori1','ori2','have intent1','have intent2','no intent1','no intent2')


    %% 
    if file < seperate_rate
        all_round_thisd_train_X_EMG=[all_round_thisd_train_X_EMG; this_round_train_EMG];
    else
        all_round_thisd_test_X_EMG=[all_round_thisd_test_X_EMG; this_round_test_EMG];
    end
    end

    
    all_round_train_X_EMG=[all_round_train_X_EMG; all_round_thisd_train_X_EMG];
    all_round_test_X_EMG=[all_round_test_X_EMG; all_round_thisd_test_X_EMG];
end


%% 分类
% sample=all_round_test_X_EMG(:,1:3);
% training=all_round_train_X_EMG(:,1:3);
% group=all_round_train_X_EMG(:,end);
% test_answer=all_round_test_X_EMG(:,end);

all_round_train_X_EMG=all_round_train_X_EMG(:,[13:end]);
all_round_test_X_EMG=all_round_test_X_EMG(:,[13:end]);

sample=all_round_test_X_EMG(:,1:end-1);
training=all_round_train_X_EMG(:,1:end-1);
group=all_round_train_X_EMG(:,end);
test_answer=all_round_test_X_EMG(:,end);

% SVM
X=training;
Y=group;
classOrder = unique(Y);
t = templateSVM('Standardize',true);
PMdl = fitcecoc(X,Y,'Holdout',0.10,'Learners',t,'ClassNames',classOrder);
Mdl = PMdl.Trained{1};           % Extract trained, compact classifier



outclass = predict(Mdl,sample);




% qda
% [outclass, SigmaHat, logDetSigma,gmeans, posterior] = QDA_classify(sample, training, group);

% KNN

% mdl = fitcknn(training, group);
% mdl = fitcknn(training, group,'OptimizeHyperparameters','auto',...
%     'HyperparameterOptimizationOptions',...
%     struct('AcquisitionFunctionName','expected-improvement-plus'),'HyperparameterOptimizationOptions', struct('UseParallel',true)) 
% outclass = predict(mdl,sample);


% simple
% outclass=[];
% for w = 1:size(all_round_test_X_EMG,1)
%     this=all_round_test_X_EMG(w,1:3);
%     [m,which]=max(abs(this));
%     if which == 1 && this(which) > 0
%         out= 1;
%     elseif which == 1 && this(which) < 0
%         out= 2;
%     end
%      if which == 2 && this(which) > 0
%         out= 3;
%      elseif which == 2 && this(which) < 0
%         out= 4;
%      end
%      if which == 3 && this(which) > 0
%         out= 5;
%      elseif which == 3 && this(which) < 0
%         out= 6;
%      end
%         outclass=[outclass out];
% end
% outclass=outclass.';


% % SVM
% %训练分类模型
% svmModel = svmtrain(train,group,'kernel_function','rbf','showplot',true);
% %分类
% classification=svmclassify(svmModel,test,'Showplot',true);


delta=outclass-test_answer;
correct=find(delta==0);
answer=length(correct)/size(sample,1)

% figure(14+seperate_rate)
% plot(outclass); hold on;
% plot(test_answer);
% legend('predict','real')


% 
% figure(55)
% plot(all_round_train_X_EMG(:,1:3)); hold on;
% plot(all_round_train_X_EMG(:,end)*0.01);

% figure(55)
% plot(all_round_test_X_EMG(:,1:3)); hold on;
% plot(all_round_test_X_EMG(:,end)*0.01);

%   %% LASSO
% rng default % For reproducibility 
% 
% % [A1,PS]=mapminmax(en_after.');
% after_all_8EMG=all_round_train_X_EMG(:,1:end-1);
% theta_real=all_round_train_X_EMG(:,end);
% [coef_1,fitinfo] = lasso(after_all_8EMG,theta_real,'Weights',ones(size(after_all_8EMG,1),1),'Alpha',0.75,'Lambda',0.005);
% % [coef_1,FitInfo] = lasso(after_all_8EMG,theta_real,'CV',10);
% % lassoPlot(B,FitInfo,'PlotType','CV');
% % legend('show') % Show legend
% figure(15+seperate_rate)
% pre_y=all_round_test_X_EMG(:,1:end-1)*coef_1+fitinfo.Intercept;
% % pre_y=after_all_8EMG*coef_1(:,1)+FitInfo.Intercept(1,1);
% 
% plot(pre_y); hold on; plot(all_round_test_X_EMG(:,end))
% legend('real angle','regression')
% 
% delta=pre_y-all_round_test_X_EMG(:,end);
% correct=find(delta==0);
% answer_qda=length(correct)/size(sample,1)
end




