close all;clear;clc;
% use GPR to predict
load('20220125good_down.mat')
%执行两次，step12

% load('20220119fial.mat');

% roundfrom1=[all_end_effector_p(:,5000:end)]; 
% roundfrom2=[all_end_effector_p(:,5000:end)]; 




STEP = 2


if STEP ==1 
roundfrom1=[all_end_effector_p(:,2200:4000)]; 
roundfrom2=[all_end_effector_p(:,2200:4000)]; 
end

% see
result=[]; count_others=1;
for look = 1:length(come_in)
    if come_in(look) == 4
        result=[result; [4 4 ...
            4 FUCKact(look) come_in(look)];];        
    else
        
        result=[result; [all_end_effector_p(1,count_others)*3 all_this_point_after(count_others)/100 ...
            output(count_others) FUCKact(look) come_in(look)];];
        count_others=count_others+1;
    end
end 
figure(18);plot(result)
legend('real position','all_this_point_after','intent','act','comein')
% 
% x=all_refer(1,8000:end); 
% y=all_refer(2,8000:end); 
% t=polyfit(x,y,1);
% z=polyval(t,x);
% zh=tan(angle_comb);
% ju=mean(x-y./zh);
% z2=(x-ju)*zh
% figure(8)
% plot(x,y,'o'); hold on;
% plot(x,z); hold on;
% plot(x,z2)

if STEP == 2
chuyi=all_end_effector_p(2,6000:8000)./tan(angle_comb);
neww=[chuyi; all_end_effector_p(2:end,6000:8000)];
roundfrom1=neww;
roundfrom2=neww;
end


%%
train_X1=[]; train_Y1=[]; test_X1=[]; test_Y1=[];
train_X2=[]; train_Y2=[]; test_X2=[]; test_Y2=[];
train_Xo=[]; train_Yo=[]; test_Xo=[]; test_Yo=[];
all_round_train_X=[];all_round_train_Y=[];
all_round_test_X=[]; all_round_test_Y=[];

start=1;list_rate_add=0:1;

%%
for file = start:2
    sep=1;
    all_round_thisd_train_X=[]; all_round_thisd_train_Y=[];
    all_round_thisd_test_X=[]; all_round_thisd_test_Y=[];

    name_g=['roundfrom' num2str(file)];
    round_this=eval(name_g);
    v=round_this(1,2:end) - round_this(1,1:end-1);
    

    
  %%  method 1
            temp_X=[];temp_Y=[];
            for shit = 1:length(v)-1
                shit;
                this = v(shit);
                next = v(shit+1);
                if this > 0
                    intent_t=1;
                else
                    intent_t=2;
                end
                if next > 0
                    intent_n=1;
                else
                    intent_n=2;
                end                
                
                
                if shit == 1
                    mark=intent_t;
                    init_t=intent_t;
                end
                
                
                temp_X=[temp_X [round_this(:,shit); intent_t;]];
                temp_Y=[temp_Y [round_this(:,shit+1); intent_t;]];
                if this*next < 0
                    hh_X=round_this(:,shit+1).*ones(3,40);
                    hh_Y=round_this(:,shit+1).*ones(3,40);
                    temp_X=[temp_X [hh_X; intent_t*ones(1,size(hh_X,2));]];
                    temp_Y=[temp_Y [hh_X; intent_t*ones(1,size(hh_X,2));]];
                end

            end
  %%  method 2
%    temp_X=[];temp_Y=[]; flag=3;count_head=0;
%    result_temp=result(2000:4000,:);
%    for shit = 2:length(round_this)
%        this_intent=result_temp(shit,4);
%        pos=result_temp(shit,1);
% 
% %        next_intent=result_temp(shit,4);
%                 if flag == 3
%                     intent_t=1;
%                 else
%                     intent_t=2;
%                 end
%         if this_intent == flag
%             if pos == 4
%                     hh_X=round_this(:,shit-1).*ones(3,40);
%                     hh_Y=round_this(:,shit-1).*ones(3,40);
%                     temp_X=[temp_X [hh_X; intent_t*ones(1,size(hh_X,2));]];
%                     temp_Y=[temp_Y [hh_X; intent_t*ones(1,size(hh_X,2));]];
%                     flag=-flag
%                     hh_X=round_this(:,shit-1).*ones(3,40);
%                     hh_Y=round_this(:,shit-1).*ones(3,40);
%                     temp_X=[temp_X [hh_X; intent_t*ones(1,size(hh_X,2));]];
%                     temp_Y=[temp_Y [hh_X; intent_t*ones(1,size(hh_X,2));]];
%                     
%                 count_head=count_head+1
%             else
%                 temp_X=[temp_X [round_this(:,shit-1); intent_t;]];
%                 temp_Y=[temp_Y [round_this(:,shit); intent_t;]];          
%             end
% 
%         else
%             count_head=0;
%         end
%    end
            
 %%           
%             figure(89)
%             plot(temp_X(1,:)); hold on; plot(temp_Y(1,:)); 
            
     if file <= sep
            
        this_round_train_X=temp_X;
        this_round_train_Y=temp_Y(1:3,:);
        all_round_thisd_train_X=[all_round_thisd_train_X; this_round_train_X.'];
        all_round_thisd_train_Y=[all_round_thisd_train_Y; this_round_train_Y.'];        

    else


        this_round_test_X=temp_X;
        this_round_test_Y=temp_Y(1:3,:);
        all_round_thisd_test_X=[all_round_thisd_test_X; this_round_test_X.'];
        all_round_thisd_test_Y=[all_round_thisd_test_Y; this_round_test_Y.'];        


    end

    all_round_train_X=[all_round_train_X; all_round_thisd_train_X];
    all_round_train_Y=[all_round_train_Y; all_round_thisd_train_Y];
    
    all_round_test_X=[all_round_test_X; all_round_thisd_test_X];
    all_round_test_Y=[all_round_test_Y; all_round_thisd_test_Y];    


end

[all_round_test_X all_round_test_Y]= delete_lessdata(all_round_test_X,all_round_test_Y);

[all_round_train_X all_round_train_Y] = delete_lessdata(all_round_train_X,all_round_train_Y);


figure(90)
plot(all_round_test_X(:,end)); hold on;
plot(all_round_test_X(:,1))
processed_X=[];







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
tic
                
        result = myGRP_xyz(each_test_file_left(:,1:end-1), each_intent, new_gprMdl_1,new_gprMdl_2,new_gprMdl_3);
toc

%         [result, nmb] = check_GPR(train_X_xyz2(:,1:end-1), each_intent, new_gprMdl_1,new_gprMdl_2,new_gprMdl_3);
    
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


    close all;
    
    figure(87)
    plot(all_result_this_rate_use)
    legend(' predict', 'real','predict_intent','real_intent')
if STEP ==2    
save('gprMdl20220127_2.mat','gprMdl_x1','gprMdl_x2','gprMdl_y1','gprMdl_y2','gprMdl_z1','gprMdl_z2')
end

if STEP ==1 
save('gprMdl20220127_1.mat','gprMdl_x1','gprMdl_x2','gprMdl_y1','gprMdl_y2','gprMdl_z1','gprMdl_z2')
end
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

