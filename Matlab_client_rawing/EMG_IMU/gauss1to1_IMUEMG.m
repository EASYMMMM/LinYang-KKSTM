close all;clear;clc;
%%
R=0.005;
kalmanz = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
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

train_X1=[]; train_Y1=[]; test_X1=[]; test_Y1=[];
train_X2=[]; train_Y2=[]; test_X2=[]; test_Y2=[];
train_Xo=[]; train_Yo=[]; test_Xo=[]; test_Yo=[];
all_round_train_X=[];all_round_train_Y=[];
all_round_test_X=[]; all_round_test_Y=[];

start=1;list_rate_add=0:1;

%%
for file = start:4
    sep=1;
    all_round_thisd_train_X=[]; all_round_thisd_train_Y=[];
    all_round_thisd_test_X=[]; all_round_thisd_test_Y=[];

    name_g=['round' num2str(file)];
    round_this=eval(name_g);
    round_this=round_this(:,500:2500);

    % 归一化
    if file == start
         [map_EMG, ps]=mapminmax(round_this(4:10,:),0,1);    
    else
         map_EMG=mapminmax('apply',round_this(4:10,:),ps);
    end
%     map_EMG=round_this(4:10,:);
%     [map_EMG,PS]=mapminmax(round_this(4:10,:));
    
    
    
    my_features=[map_EMG(1,:)-map_EMG(2,:); map_EMG(3,:)-map_EMG(4,:); map_EMG(5,:)-map_EMG(7,:);].';
    F=round_this(1,:).';
    
    
    
    
    
    max_F=max(F); min_F=min(F); 
%     threhold=(max_F-min_F)/2.5;
%     intent1=find(F>max_F-threhold);
%     intent2=find(F<min_F+threhold);


    
    fea1=my_features(:,1).';
    fea2=my_features(:,2).';
    fea3=my_features(:,3).';
    fea4=map_EMG(6,:);
 %% ABY   
    jerk=[];
    for nmb = 1:length(F)-1
        this=F(nmb);
        next=F(nmb+1);
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
    all_y=all_y-all_y(1);
    figure(1+file)
    plot(all_y)
    
  %%  
  
    sele_st=1; sele_end=7;
    if file <= sep
            intent1=find(F>20);
            intent2=find(F<-20);
            intent_o=1:length(F); intent_o([intent1; intent2])=[];
            F_1=F(intent1);
            F_2=F(intent2);
            F_other=F(intent_o);

        intent1(end)=[];intent2(end)=[];


        this_round_train_X=[all_y(intent1); map_EMG(5:6,intent1); 1*ones(1,length(intent1));];
        this_round_train_Y=[all_y(intent1+1); map_EMG(5:6,intent1+1); ];
        all_round_thisd_train_X=[all_round_thisd_train_X; this_round_train_X.'];
        all_round_thisd_train_Y=[all_round_thisd_train_Y; this_round_train_Y.'];        

        this_round_train_X=[all_y(intent2); map_EMG(5:6,intent2);  2*ones(1,length(intent2));];
        this_round_train_Y=[all_y(intent2+1); map_EMG(5:6,intent2+1);];
        all_round_thisd_train_X=[all_round_thisd_train_X; this_round_train_X.'];
        all_round_thisd_train_Y=[all_round_thisd_train_Y; this_round_train_Y.'];     
    else
        
            intent1=find(F>0);
            intent2=find(F<0);
            intent_o=1:length(F); intent_o([intent1; intent2])=[];
            F_1=F(intent1);
            F_2=F(intent2);
            F_other=F(intent_o); 
            
        intent1(end)=[];intent2(end)=[];


        this_round_test_X=[all_y(intent1); map_EMG(5:6,intent1); 1*ones(1,length(intent1));];
        this_round_test_Y=[all_y(intent1+1); map_EMG(5:6,intent1+1); ];
        all_round_thisd_test_X=[all_round_thisd_test_X; this_round_test_X.'];
        all_round_thisd_test_Y=[all_round_thisd_test_Y; this_round_test_Y.'];        

        this_round_test_X=[all_y(intent2); map_EMG(5:6,intent2); 2*ones(1,length(intent2));];
        this_round_test_Y=[all_y(intent2+1); map_EMG(5:6,intent2+1);];
        all_round_thisd_test_X=[all_round_thisd_test_X; this_round_test_X.'];
        all_round_thisd_test_Y=[all_round_thisd_test_Y; this_round_test_Y.'];  
 

%% predict in loop
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
%     plot(F);
%     legend('pre','KkK','real')

    end

    all_round_train_X=[all_round_train_X; all_round_thisd_train_X];
    all_round_train_Y=[all_round_train_Y; all_round_thisd_train_Y];
    
    all_round_test_X=[all_round_test_X; all_round_thisd_test_X];
    all_round_test_Y=[all_round_test_Y; all_round_thisd_test_Y];    


end


for g = 1:2
        expr_g=['which' num2str(g) ' = find(all_round_train_X(:,end)==' num2str(g) '); '];
        eval(expr_g);
        expr_g2=['train_X_xyz' num2str(g) ' = all_round_train_X(which' num2str(g) ',1:3);'];
        eval(expr_g2);
        expr_g3=['train_Y_xyz' num2str(g) ' = all_round_train_Y(which' num2str(g) ',1:3);'];
        eval(expr_g3);
end



train_X_xyz1 = [train_X_xyz1 1*ones(size(train_X_xyz1,1),1)];
train_X_xyz2 = [train_X_xyz2 2*ones(size(train_X_xyz2,1),1)];

gprMdl_x1 = fitrgp(train_X_xyz1,train_Y_xyz1(:,1));
gprMdl_y1 = fitrgp(train_X_xyz1,train_Y_xyz1(:,2));
gprMdl_z1 = fitrgp(train_X_xyz1,train_Y_xyz1(:,3));
%% predict from the older mdl
figure(41)
[ypred_f, ~, yci_f] = predict(gprMdl_x1, train_X_xyz1 );

title('no data, not much confidence on the right');
hold on;
plot(train_X_xyz1(:,1) ,gprMdl_x1.Y,'r.');
plot(train_X_xyz1(:,1) ,ypred_f, 'y.');
plot(train_X_xyz1(:,1), yci_f(:,1),'k:');
plot(train_X_xyz1(:,1), yci_f(:,2),'k:');
max_range=max(train_X_xyz1(:,1)); min_range=min(train_X_xyz1(:,1));
plot([min_range, max_range] , [min_range, max_range] ,'g')


gprMdl_x2 = fitrgp(train_X_xyz2,train_Y_xyz2(:,1));
gprMdl_y2 = fitrgp(train_X_xyz2,train_Y_xyz2(:,2));
gprMdl_z2 = fitrgp(train_X_xyz2,train_Y_xyz2(:,3));

%% predict from the older mdl
figure(42)
[ypred_f, ~, yci_f] = predict(gprMdl_x2, train_X_xyz2 );

title('no data, not much confidence on the right');
hold on;
plot(train_X_xyz2(:,1) ,gprMdl_x2.Y,'r.');
plot(train_X_xyz2(:,1) ,ypred_f, 'y.');
plot(train_X_xyz2(:,1), yci_f(:,1),'k:');
plot(train_X_xyz2(:,1), yci_f(:,2),'k:');
max_range=max(train_X_xyz2(:,1)); min_range=min(train_X_xyz2(:,1));
% jj=min_range-20:0.1:max_range+20;
% [ypred_f, ~, yci_f] = predict(gprMdl_x2, jj );
% plot(jj ,ypred_f, 'y.');

plot([min_range, max_range] , [min_range, max_range] ,'g')
% %% 
% new_gprMdl_1
% figure(43)
% [ypred_f, ~, yci_f] = predict(new_gprMdl_1, new_gprMdl_1.X);
% 
% title('after adding new data to train');
% hold on;
% plot(new_gprMdl_1.X(:,1) ,new_gprMdl_1.Y,'r.');
% plot(new_gprMdl_1.X(:,1) ,ypred_f, 'y.');
% % plot(train_X_xyz2(:,1), yci_f(:,1),'k:');
% % plot(train_X_xyz2(:,1), yci_f(:,2),'k:');
% max_range=max(train_X_xyz2(:,1)); min_range=min(train_X_xyz2(:,1));
% % jj=min_range-20:0.1:max_range+20;
% % [ypred_f, ~, yci_f] = predict(gprMdl_x2, jj );
% % plot(jj ,ypred_f, 'y.');
% 
% plot([min_range, max_range] , [min_range, max_range] ,'g')
% %% 
% new_gprMdl_1
% figure(44)
% [ypred_f, ~, yci_f] = predict(new_gprMdl_1, train_X_xyz2);
% 
% title('after adding new data to train');
% hold on;
% plot(train_X_xyz2(:,1) ,gprMdl_x2.Y,'r.');
% plot(train_X_xyz2(:,1) ,ypred_f, 'y.');
% % plot(train_X_xyz2(:,1), yci_f(:,1),'k:');
% % plot(train_X_xyz2(:,1), yci_f(:,2),'k:');
% max_range=max(train_X_xyz2(:,1)); min_range=min(train_X_xyz2(:,1));
% % jj=min_range-20:0.1:max_range+20;
% % [ypred_f, ~, yci_f] = predict(gprMdl_x2, jj );
% % plot(jj ,ypred_f, 'y.');
% 
% plot([min_range, max_range] , [min_range, max_range] ,'g')





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


this_round_corr=[];
all_MSE_this_rate=[];denominator=length(list_rate_add);result=0;all_result_this_rate=[];all_result_this_rate_use=[];
for rate_add = list_rate_add
    
    all_MSE_thisrate=[];this_round_result=[];this_round_result_use=[];
    for each_test = num_all_intent
        each_test; % 第几次
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
             each_test_file_left_draw=each_test_file_left(:,1);
              new_gprMdl_1 = updateGPRMdl(gprMdl_x1, temp_combine_X,  temp_combine_Y(:,1));
              new_gprMdl_2 = updateGPRMdl(gprMdl_y1, temp_combine_X,  temp_combine_Y(:,2));
              new_gprMdl_3 = updateGPRMdl(gprMdl_z1, temp_combine_X,  temp_combine_Y(:,3));
              gprMdl_draw=new_gprMdl_1;
        elseif each_intent == 2
              temp_combine_X=[each_test_file_add; train_X_xyz2;];
             temp_combine_Y=[each_test_file_Y_add; train_Y_xyz2;];          
              temp_combine_X_draw=temp_combine_X(:,1);
             temp_combine_Y_draw=temp_combine_Y(:,1);     
             each_test_file_left_draw=each_test_file_left(:,1);
              new_gprMdl_1 = updateGPRMdl(gprMdl_x2, temp_combine_X,  temp_combine_Y(:,1));
              new_gprMdl_2 = updateGPRMdl(gprMdl_y2, temp_combine_X,  temp_combine_Y(:,2));
              new_gprMdl_3 = updateGPRMdl(gprMdl_z2, temp_combine_X,  temp_combine_Y(:,3));
              gprMdl_draw=new_gprMdl_1;
        end

                
        result = myGRP_xyz(each_test_file_left(:,1:end-1), each_intent, new_gprMdl_1,new_gprMdl_2,new_gprMdl_3);


        [result, nmb] = check_GPR(train_X_xyz2(:,1:end-1), each_intent, new_gprMdl_1,new_gprMdl_2,new_gprMdl_3);
    
        result_xyz=result(:,1:3);
        real_xyz=each_test_file_Y_left(1:end,1:3);
        
        mse = sum((real_xyz - result_xyz).^2)./size(result,1);
    

        avg_mse=sum(mse)/3;
        all_MSE_thisrate=[all_MSE_thisrate avg_mse];
        each_com=[result_xyz real_xyz];
        this_round_result=[this_round_result; each_com;];

            each_com_use= [result_xyz(:,1) real_xyz(:,1)];

        
        pre_corr = pre_correct(each_com_use, each_intent);
            
        each_com_use=[each_com_use pre_corr each_intent*ones(size(each_com_use,1),1)];
        this_round_result_use=[this_round_result_use; each_com_use;];
        
        
    end
    all_MSE_this_rate=[all_MSE_this_rate; sum(all_MSE_thisrate)/length(all_MSE_thisrate);];
    all_result_this_rate=[all_result_this_rate; this_round_result; zeros(1,6);];
    
    
corr_total2=find(this_round_result_use(:,3) == this_round_result_use(:,4));
sb2=length(corr_total2);
rate2=sb2/size(this_round_result_use,1);
    this_round_corr=[this_round_corr; rate2;];
    
    all_result_this_rate_use=[all_result_this_rate_use; this_round_result_use; zeros(1,4);];
    
    % 不同的0隔开了，不同的测试集，在一个相同的训练adding rate
end


%     close all;
%     
%     figure(87)
%     plot(all_result_this_rate_use)
%     legend(' predict', 'real','predict_intent','real_intent')
    

% 
% figure(65)
% rate_fig=(list_seperate_rate - 1)./(9-list_seperate_rate)
% percent_adding=list_rate_add*100/(length(list_rate_add)+1)
% [X, Y] = meshgrid(rate_fig,percent_adding);
% 
% mesh(X,Y,ALL_corr)
% ylabel('percent for adding in this test trival / %')
% xlabel('rate of train to test')
% zlabel('正确率')

