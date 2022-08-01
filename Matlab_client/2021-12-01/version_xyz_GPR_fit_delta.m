close all;clear;clc;
% this file is used to divide the raw series of data into labeled data.
% 2021-12-07
% round1= load ('round1.mat').sychronize;
% round2= load ('round2.mat').sychronize;  % ???
% round3= load ('round3.mat').sychronize;
% round4= load ('round4.mat').sychronize;
% round5= load ('round5.mat').sychronize;
% round6= load ('round6.mat').sychronize;
% round7= load ('round7.mat').sychronize;
% round8= load ('round8.mat').sychronize;
% round9= load ('round9.mat').sychronize;
% round10= load ('round10.mat').sychronize;
% round11= load ("round11.mat").sychronize;
% % 左右上下后前——123456
%%
round1= load ('round1.mat').sychronize;
round2= load ('round7.mat').sychronize;  % ???
round3= load ('round3.mat').sychronize;
round4= load ('round4.mat').sychronize;
round5= load ('round5.mat').sychronize;
round6= load ('round6.mat').sychronize;
% round7= load ('round7.mat').sychronize;
% round8= load ('round8.mat').sychronize;
% round9= load ('round9.mat').sychronize;
% round10= load ('round10.mat').sychronize;
% round11= load ("round11.mat").sychronize;
% 左右上下后前——123456

%%
% figure(11)% for (:,8)), 298~769
% for f = 1:11
%         expr_g=['plot(round' num2str(f) '(:,7));']; 
%         value_g=eval(expr_g);
%         hold on;
% end
% figure(12)% for (:,8)), 298~769
% for f = 1:11
%         expr_g=['plot(round' num2str(f) '(:,8));']; 
%         value_g=eval(expr_g);
%         hold on;
% end
% figure(13)% for (:,8)), 298~769
% for f = 1:11
%         expr_g=['plot(round' num2str(f) '(:,9));']; 
%         value_g=eval(expr_g);
%         hold on;
% end
list_seperate_rate=2:1:2; list_rate_add=0:2;see_slope=0;
ALL_MSE=[];ALL_result=[];ALL_result_use=[];
for seperate_rate = list_seperate_rate
    seperate_rate
%%
all_round_train_X=[]; all_round_train_Y=[];all_round_test_Y=[];all_round_test_X=[];
for FLAG = 1:3
    if FLAG == 2 % y
        wrist_direction=8; lower_bound=298; upper_bound=769; label1=3; label2=4; howmany = 800;
    elseif FLAG == 3 % z
        wrist_direction=9; lower_bound=60; upper_bound=326; label1=5; label2=6; howmany = 800;
    elseif FLAG == 1  % x
        wrist_direction=7; lower_bound=615; upper_bound=1000; label1=1; label2=2; howmany = 800;
    end
    
    
%%  
all_round_thisd_train_X=[]; all_round_thisd_train_Y=[];all_round_thisd_test_X=[]; all_round_thisd_test_Y=[];
for file = 1:2

    
    name_g=['round' num2str(file)];
    round_this=eval(name_g);
    
    
%     figure(file+20); %——注释掉，调试用的
    t=1:length(round_this(:,wrist_direction));
    p=polyfit(t,round_this(:,wrist_direction),20);
    yi=polyval(p,t);

%     plot(t,round_this(:,wrist_direction),':o',t,yi,'-*'); hold on; %——注释掉，调试用的
    
    slope=[];
    for i = 1:length(yi)-1
        this = yi(i);
        next=yi(i+1);
        delta=next-this;
        slope=[slope delta];
    end
%     plot(slope*10)  %——注释掉，调试用的

    sorted_slope=sort(abs(slope));
    threhold=sorted_slope(howmany);
    which_want=find(abs(slope) > threhold);
    new_which_want=[];
    for z = 1:length(which_want)
        oo=which_want(z);
        if oo < upper_bound && oo > lower_bound
            new_which_want=[new_which_want oo];
        end
    end

    want=yi(new_which_want);
    

    
    
    if FLAG == 1
        count=0;simple_want=[];temp_want=[];
        inv_new_which_want=fliplr(new_which_want);
        for k = 1:length(inv_new_which_want)-1
            this_want=inv_new_which_want(k);
            next_want=inv_new_which_want(k+1);
            if this_want-1 == next_want && count < 2 
                temp_want=[temp_want this_want];

            else
                if length(temp_want) > 20
                    simple_want=[simple_want temp_want];
                    count=count+1;
                end 
                temp_want=[];
                
            end
        end
    new_which_want=fliplr(simple_want);
    want=yi(new_which_want);       
        
    end


%     
%      %——注释掉，调试用的
%     figure(4+file);
%     plot(t,round_this(:,wrist_direction),':o',t,yi,'-*'); hold on;
%     plot(new_which_want,want,'o','MarkerSize',10)
    
    
    round_this_EMG_map=mapminmax(round_this(:,13:21).',0,1).';
    new_this_round=[round_this(:,1:12) round_this_EMG_map round_this(:,22:end)];
    this_round_train_X=[];this_round_train_Y=[];this_round_test_X=[];this_round_test_Y=[];
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
       
        if this_slope > 0
            this_round_train_X=[this_round_train_X; [yx(each_slope) yy(each_slope) yz(each_slope)  label1];];
        else
            this_round_train_X=[this_round_train_X; [yx(each_slope) yy(each_slope) yz(each_slope) label2];];
        end
        
        if this_slope > 0
            this_round_test_X=[this_round_test_X; [new_this_round(each_slope,7:9) label1];];
        else
            this_round_test_X=[this_round_test_X; [new_this_round(each_slope,7:9) label2];];
        end
        
        this_round_updown_xyz_Y=[this_round_updown_xyz_Y; new_this_round(each_slope+1,7:9);];
        
        this_round_EMG_each_Y= [[new_this_round(each_slope+1,1+12)-new_this_round(each_slope+1,2+12)]...
            [new_this_round(each_slope+1,3+12)-new_this_round(each_slope+1,4+12)]...
            [new_this_round(each_slope+1,5+12)-new_this_round(each_slope+1,7+12)]...
            [new_this_round(each_slope+1,8+12)-new_this_round(each_slope+1,9+12)]...
            [new_this_round(each_slope+1,6+12)]];
        this_round_EMG_Y=[this_round_EMG_Y; this_round_EMG_each_Y;];
        
         this_round_pass0_each_Y= [[new_this_round(each_slope+1,1+21)-new_this_round(each_slope+1,2+21)]...
            [new_this_round(each_slope+1,3+21)-new_this_round(each_slope+1,4+21)]...
            [new_this_round(each_slope+1,5+21)-new_this_round(each_slope+1,7+21)]...
            [new_this_round(each_slope+1,8+21)-new_this_round(each_slope+1,9+21)]...
            [new_this_round(each_slope+1,6+21)]];
        this_round_pass0_Y=[this_round_pass0_Y; this_round_pass0_each_Y;];
        
        this_round_test_Y=[this_round_test_Y; new_this_round(each_slope+1,7:9);];
        
        this_round_train_Y=[this_round_train_Y; [yx(each_slope+1) yy(each_slope+1) yz(each_slope+1)] ;];
    end
    
    %% 
    if file < seperate_rate
        all_round_thisd_train_X=[all_round_thisd_train_X; this_round_train_X];
        all_round_thisd_train_Y=[all_round_thisd_train_Y; this_round_train_Y];
    else
        all_round_thisd_test_X=[all_round_thisd_test_X; this_round_test_X];
        all_round_thisd_test_Y=[all_round_thisd_test_Y; this_round_test_Y];  
    end
end



    all_round_train_X=[all_round_train_X; all_round_thisd_train_X];
    all_round_train_Y=[all_round_train_Y; all_round_thisd_train_Y];
    
    all_round_test_X=[all_round_test_X; all_round_thisd_test_X];
    all_round_test_Y=[all_round_test_Y; all_round_thisd_test_Y];    
%     pause(10);
end
%% delta
init_train_X=all_round_train_X(1,1:end-1);

    all_round_train_X=[[all_round_train_X(:,1:end-1) - all_round_train_X(1,1:end-1)] all_round_train_X(:,end)];
    all_round_train_Y=[all_round_train_Y - all_round_train_Y(1,:)];
    
    all_round_test_X=[[all_round_test_X(:,1:end-1) - all_round_test_X(1,1:end-1)] all_round_test_X(:,end)];
    all_round_test_Y=[all_round_test_Y - all_round_test_Y(1,:)];


%% Train INIT
X_train=all_round_train_X;
Y_train=all_round_train_Y;


for g = 1:6
        expr_g=['which' num2str(g) ' = find(all_round_train_X(:,end)==' num2str(g) '); '];
        eval(expr_g);
        expr_g2=['train_X_xyz' num2str(g) ' = all_round_train_X(which' num2str(g) ',1:3);'];
        eval(expr_g2);
        expr_g3=['train_Y_xyz' num2str(g) ' = all_round_train_Y(which' num2str(g) ',1:3);'];
        eval(expr_g3);
end

% plot(train_Y_xyz1(:,1)); hold on; 
% plot(train_X_xyz1(:,1));


train_X_xyz1 = [train_X_xyz1 1*ones(size(train_X_xyz1,1),1)];
train_X_xyz2 = [train_X_xyz2 2*ones(size(train_X_xyz2,1),1)];
train_X_xyz3 = [train_X_xyz3 3*ones(size(train_X_xyz3,1),1)];

train_X_xyz4 = [train_X_xyz4 4*ones(size(train_X_xyz4,1),1)];
train_X_xyz5 = [train_X_xyz5 5*ones(size(train_X_xyz5,1),1)];
train_X_xyz6 = [train_X_xyz6 6*ones(size(train_X_xyz6,1),1)];

rng default

gprMdl_x1 = fitrgp(train_X_xyz1,train_Y_xyz1(:,1),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_y1 = fitrgp(train_X_xyz1,train_Y_xyz1(:,2),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_z1 = fitrgp(train_X_xyz1,train_Y_xyz1(:,3),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));

gprMdl_x2 = fitrgp(train_X_xyz2,train_Y_xyz2(:,1),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_y2 = fitrgp(train_X_xyz2,train_Y_xyz2(:,2),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_z2 = fitrgp(train_X_xyz2,train_Y_xyz2(:,3),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));

gprMdl_x3 = fitrgp(train_X_xyz3,train_Y_xyz3(:,1),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_y3 = fitrgp(train_X_xyz3,train_Y_xyz3(:,2),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_z3 = fitrgp(train_X_xyz3,train_Y_xyz3(:,3),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));

gprMdl_x4 = fitrgp(train_X_xyz4,train_Y_xyz4(:,1),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_y4 = fitrgp(train_X_xyz4,train_Y_xyz4(:,2),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_z4 = fitrgp(train_X_xyz4,train_Y_xyz4(:,3),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));

gprMdl_x5 = fitrgp(train_X_xyz5,train_Y_xyz5(:,1),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_y5 = fitrgp(train_X_xyz5,train_Y_xyz5(:,2),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_z5 = fitrgp(train_X_xyz5,train_Y_xyz5(:,3),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));

gprMdl_x6 = fitrgp(train_X_xyz6,train_Y_xyz6(:,1),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_y6 = fitrgp(train_X_xyz6,train_Y_xyz6(:,2),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));
gprMdl_z6 = fitrgp(train_X_xyz6,train_Y_xyz6(:,3),'KernelFunction','squaredexponential','ComputationMethod','v','ActiveSetMethod','likelihood','OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',struct('AcquisitionFunctionName','expected-improvement-plus'));

%% seeee~

% close all;
% each_test=1;
% each_intent=all_intent(each_test)
% which_each_test_file = find(after_all_test_X(:,end) == each_test);
% each_test_file = after_all_test_X(which_each_test_file,:); % this is each trival, 14 with intent
% 
% xyz=1; start=1
% 
% 
% figure(38)
% result = myGRP_xyz(train_X_xyz2(:,1:end-1), each_intent, gprMdl_x2,gprMdl_y2,gprMdl_z2);
% plot(train_X_xyz2(:,xyz)); hold on; plot(result(:,xyz))
% 
% figure(38)
% result = myGRP_xyz(each_test_file(start:end,1:end-1), each_intent, gprMdl_x2,gprMdl_y2,gprMdl_z2);
% plot(each_test_file(start:end,xyz)); hold on; plot(result(:,xyz))
% legend('训练集主轴真实结果','训练集直接预测','测试集主轴真实结果','测试集直接预测')
% 

%% 寻找试次
% all_round_test_X=X_train;


all_intent=[];
for trival = 1:size(all_round_test_X,1)-1
    this_intent = all_round_test_X(trival,end);
    next_intent = all_round_test_X(trival+1,end);
    if this_intent ~= next_intent
        all_intent=[all_intent this_intent];
    end
end
all_intent=[all_intent next_intent];
num_all_intent=1:length(all_intent);
count_intent=1; after_all_test_X=[]; 


for trival2 = 1:size(all_round_test_X,1)
    this_intent = all_round_test_X(trival2,end);
    if this_intent ~= all_intent(count_intent)
        count_intent=count_intent+1;
    end
        after_all_test_X=[after_all_test_X; [all_round_test_X(trival2,1:end-1) count_intent]; ];       
end


all_MSE_this_rate=[];denominator=length(list_rate_add);result=0;all_result_this_rate=[];all_result_this_rate_use=[];
for rate_add = list_rate_add
    
    all_MSE_thisrate=[];this_round_result=[];this_round_result_use=[];
    for each_test = num_all_intent
        each_test % 第几次
        each_intent=all_intent(each_test);
        which_each_test_file = find(after_all_test_X(:,end) == each_test);
        each_test_file = after_all_test_X(which_each_test_file,:); % this is each trival, 14 with intent
        % seperate data
        bound=floor(size(each_test_file,1)*rate_add/denominator)+1;
        each_test_file_add=[each_test_file(1:bound,1:end-1)    each_intent*ones(size(each_test_file(1:bound,:),1),1)] ;
   
        each_test_file_left=each_test_file(bound:end,:);

        each_test_file_Y = all_round_test_Y(which_each_test_file,1:end);
        each_test_file_Y_add=each_test_file_Y(1:bound,:);
        each_test_file_Y_left=each_test_file_Y(bound:end,:);       
        % update GPR
%         
%     each_test_file_add=[[each_test_file_add(:,1:end-1) - each_test_file_add(1,1:end-1)] each_test_file_add(:,end)];
%     each_test_file_Y_add=[each_test_file_Y_add - each_test_file_Y_add(1,:)];
%     
%     each_test_file_left=[[each_test_file_left(:,1:end-1) - each_test_file_left(1,1:end-1)] each_test_file_left(:,end)];
%     each_test_file_Y_left=[each_test_file_Y_left - each_test_file_Y_left(1,:)];    

        if each_intent == 1 
            temp_combine_X=[each_test_file_add; train_X_xyz1;];
             temp_combine_Y=[each_test_file_Y_add; train_Y_xyz1;];
             temp_combine_X_draw=temp_combine_X(:,1);
             temp_combine_Y_draw=temp_combine_Y(:,1);
             
              new_gprMdl_1 = updateGPRMdl(gprMdl_x1, temp_combine_X,  temp_combine_Y(:,1));
              new_gprMdl_2 = updateGPRMdl(gprMdl_y1, temp_combine_X,  temp_combine_Y(:,2));
              new_gprMdl_3 = updateGPRMdl(gprMdl_z1, temp_combine_X,  temp_combine_Y(:,3));
        elseif each_intent == 2
              temp_combine_X=[each_test_file_add; train_X_xyz2;];
             temp_combine_Y=[each_test_file_Y_add; train_Y_xyz2;];          
              temp_combine_X_draw=temp_combine_X(:,1);
             temp_combine_Y_draw=temp_combine_Y(:,1);           
              new_gprMdl_1 = updateGPRMdl(gprMdl_x2, temp_combine_X,  temp_combine_Y(:,1));
              new_gprMdl_2 = updateGPRMdl(gprMdl_y2, temp_combine_X,  temp_combine_Y(:,2));
              new_gprMdl_3 = updateGPRMdl(gprMdl_z2, temp_combine_X,  temp_combine_Y(:,3));
        elseif each_intent == 3
             temp_combine_X=[each_test_file_add; train_X_xyz3;];
             temp_combine_Y=[each_test_file_Y_add; train_Y_xyz3;];         
             temp_combine_X_draw=temp_combine_X(:,2);
             temp_combine_Y_draw=temp_combine_Y(:,2);             
              new_gprMdl_1 = updateGPRMdl(gprMdl_x3, temp_combine_X,  temp_combine_Y(:,1));
              new_gprMdl_2 = updateGPRMdl(gprMdl_y3, temp_combine_X,  temp_combine_Y(:,2));
              new_gprMdl_3 = updateGPRMdl(gprMdl_z3, temp_combine_X,  temp_combine_Y(:,3));
              
        elseif each_intent == 4
             temp_combine_X=[each_test_file_add; train_X_xyz4;];
             temp_combine_Y=[each_test_file_Y_add; train_Y_xyz4;];         
             temp_combine_X_draw=temp_combine_X(:,2);
             temp_combine_Y_draw=temp_combine_Y(:,2);             
              new_gprMdl_1 = updateGPRMdl(gprMdl_x4, temp_combine_X,  temp_combine_Y(:,1));
              new_gprMdl_2 = updateGPRMdl(gprMdl_y4, temp_combine_X,  temp_combine_Y(:,2));
              new_gprMdl_3 = updateGPRMdl(gprMdl_z4, temp_combine_X,  temp_combine_Y(:,3));
        elseif each_intent == 5
             temp_combine_X=[each_test_file_add; train_X_xyz5;];
             temp_combine_Y=[each_test_file_Y_add; train_Y_xyz5;];         
             temp_combine_X_draw=temp_combine_X(:,3);
             temp_combine_Y_draw=temp_combine_Y(:,3);             
              new_gprMdl_1 = updateGPRMdl(gprMdl_x5, temp_combine_X,  temp_combine_Y(:,1));
              new_gprMdl_2 = updateGPRMdl(gprMdl_y5, temp_combine_X,  temp_combine_Y(:,2));
              new_gprMdl_3 = updateGPRMdl(gprMdl_z5, temp_combine_X,  temp_combine_Y(:,3));
        elseif each_intent == 6
             temp_combine_X=[each_test_file_add; train_X_xyz6;];
             temp_combine_Y=[each_test_file_Y_add; train_Y_xyz6;];         
             temp_combine_X_draw=temp_combine_X(:,3);
             temp_combine_Y_draw=temp_combine_Y(:,3);
              new_gprMdl_1 = updateGPRMdl(gprMdl_x6, temp_combine_X,  temp_combine_Y(:,1));
              new_gprMdl_2 = updateGPRMdl(gprMdl_y6, temp_combine_X,  temp_combine_Y(:,2));
              new_gprMdl_3 = updateGPRMdl(gprMdl_z6, temp_combine_X,  temp_combine_Y(:,3));
        end

        result = myGRP_xyz(each_test_file_left(:,1:end-1), each_intent, new_gprMdl_1,new_gprMdl_2,new_gprMdl_3);
       
        result_xyz=result(:,1:3);
        real_xyz=each_test_file_Y_left(1:end,1:3);
        
        mse = sum((real_xyz - result_xyz).^2)./size(result,1);
    
% % 
% % ???
% figure(233)
% % plot(ypred); hold on; plot( Y_train(:,2))
%    res = myGRP(each_test_file_left(:,1:end-1), 1, gprMdl_1, gprMdl_2, gprMdl_3, gprMdl_4, gprMdl_5, gprMdl_6, gprMdl_7, gprMdl_8, gprMdl_9, gprMdl_10, gprMdl_11, gprMdl_12, gprMdl_13);

% % see slope for each add
see_slope=see_slope+1;
figure(1+see_slope)
plot(temp_combine_X_draw,temp_combine_Y_draw,'o')




%    new_res = myGRP(each_test_file_left(:,1:end-1), 1, new_gprMdl_1,new_gprMdl_2,new_gprMdl_3,new_gprMdl_4,new_gprMdl_5,new_gprMdl_6,new_gprMdl_7,new_gprMdl_8,new_gprMdl_9,new_gprMdl_10,new_gprMdl_11,new_gprMdl_12,new_gprMdl_13);
%   fuck = real_xyz(:,1);
%    plot(res(:,1)); hold on; plot(fuck); hold on; plot(new_res(:,1));
%     legend('res','real','new')     

        
        avg_mse=sum(mse)/3;
        all_MSE_thisrate=[all_MSE_thisrate avg_mse];
        each_com=[result_xyz real_xyz];
        this_round_result=[this_round_result; each_com;];
        if each_intent == 1 || each_intent == 2
            each_com_use= [result_xyz(:,1) real_xyz(:,1)];
        elseif  each_intent == 3 || each_intent == 4
            each_com_use= [result_xyz(:,2) real_xyz(:,2)];       
        elseif each_intent == 5 || each_intent == 6
            each_com_use= [result_xyz(:,3) real_xyz(:,3)];
        end
        each_com_use=[each_com_use each_intent*ones(size(each_com_use,1),1)];
        this_round_result_use=[this_round_result_use; each_com_use;];
        
        
    end
    all_MSE_this_rate=[all_MSE_this_rate; sum(all_MSE_thisrate)/length(all_MSE_thisrate);];
    all_result_this_rate=[all_result_this_rate; this_round_result; zeros(1,6);];
    
    
    
    all_result_this_rate_use=[all_result_this_rate_use; this_round_result_use; zeros(1,3);];
    
    % 不同的0隔开了，不同的测试集，在一个相同的训练adding rate
end


ALL_MSE=[ALL_MSE all_MSE_this_rate];
ALL_result=[ALL_result; all_result_this_rate; ones(1,6);];

ALL_result_use=[ALL_result_use; all_result_this_rate_use; ones(1,3);];
end

% close all;
% 
% figure(87)
% plot(ALL_result_use)
% legend(' predict', 'real','intent')


% figure(66)
% rate=(list_seperate_rate - 1)./(7-list_seperate_rate)
% percent_adding=list_rate_add*100/(length(list_rate_add)+1)
% [X, Y] = meshgrid(rate,percent_adding);
% 
% mesh(X,Y,ALL_MSE)
% ylabel('percent for adding in this test trival / %')
% xlabel('rate of train to test')
% zlabel('average MSE')
% 
% % save('2021-12-08')
% % 
% 
% all_0=all(ALL_result == 0,2);
% all_1=all(ALL_result == 1,2);
% first_round_list=find(all_0 == 1, 3);
% first_round=first_round_list(end-1:end)
% 
% figure(32)
% plot(ALL_result(1:end,1),'r.'); hold on; plot(ALL_result(1:end,1+3),'b.');
% legend('predict x','True value x');
% figure(33)
% plot(ALL_result(1:end,2),'r.'); hold on; plot(ALL_result(1:end, 2+3),'b.');
% legend('predict y','True value y');
% figure(34)
% plot(ALL_result(1:end,3),'r.'); hold on; plot(ALL_result(1:end, 3+3),'b.');
% legend('predict z','True value z');
% 
% figure(32)
% plot(ALL_result(first_round(1):first_round(2),1)); hold on; plot(ALL_result(first_round(1):first_round(2),1+3));
% legend('predict x','True value x');
% figure(33)
% plot(ALL_result(first_round(1):first_round(2),2)); hold on; plot(ALL_result(first_round(1):first_round(2), 2+3));
% legend('predict y','True value y');
% figure(34)
% plot(ALL_result(first_round(1):first_round(2),3)); hold on; plot(ALL_result(first_round(1):first_round(2), 3+3));
% legend('predict z','True value z');

% hhh=[1 2 3; 0 0 0; 1 4 5;];
% all(hhh==0,2)