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
round2= load ('round2.mat').sychronize;  % ???
round3= load ('round3.mat').sychronize;
round4= load ('round4.mat').sychronize;
round5= load ('round5.mat').sychronize;
round6= load ('round6.mat').sychronize;
round7= load ('round7.mat').sychronize;
round8= load ('round8.mat').sychronize;
round9= load ('round9.mat').sychronize;
round10= load ('round10.mat').sychronize;
round11= load ("round11.mat").sychronize;
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
list_seperate_rate=100; count_see=1;
ALL_MSE=[];ALL_result=[];ALL_result_use=[];
for seperate_rate = list_seperate_rate
    seperate_rate
%%
all_round_train_X=[]; all_round_train_Y=[];all_round_test_Y=[];all_round_test_X=[];
for FLAG = 1:3
%     if FLAG == 2 % y up1 down 2 
%         wrist_direction=8; lower_bound=298; upper_bound=769; label1=1; label2=2; howmany = 800;
%     elseif FLAG == 3 % z left 3 right 4
%         wrist_direction=9; lower_bound=60; upper_bound=326; label1=3; label2=4; howmany = 800;
%     elseif FLAG == 1  % x for 1 back 2
%         wrist_direction=7; lower_bound=615; upper_bound=1000; label1=1; label2=2; howmany = 800;
%     end
    if FLAG == 2 % y
        wrist_direction=8; lower_bound=298; upper_bound=769; label1=3; label2=4; howmany = 800;
    elseif FLAG == 3 % z
        wrist_direction=9; lower_bound=60; upper_bound=326; label1=5; label2=6; howmany = 800;
    elseif FLAG == 1  % x
        wrist_direction=7; lower_bound=615; upper_bound=1000; label1=1; label2=2; howmany = 800;
    end    
    
%%  
all_round_thisd_train_X=[]; all_round_thisd_train_Y=[];all_round_thisd_test_X=[]; all_round_thisd_test_Y=[];
for file = 1:11

    
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
%     figure(40+file);
%     plot(t,round_this(:,wrist_direction),':o',t,yi,'-*'); hold on;
%     plot(new_which_want,want,'o','MarkerSize',10)
%      

    
%     round_this_EMG_map=mapminmax(round_this(:,13:21).',0,1).';
    round_this_EMG_map=round_this(:,13:21);
    
    new_this_round=[round_this(:,1:12) round_this_EMG_map round_this(:,22:end)];
    this_round_train_X=[];this_round_train_Y=[];
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
  
        
        if this_slope > 0
            this_round_train_X=[this_round_train_X; [new_this_round(each_slope,7:9) this_round_EMG_each_X this_round_pass0_each_X label1];];
        else
            this_round_train_X=[this_round_train_X; [new_this_round(each_slope,7:9) this_round_EMG_each_X this_round_pass0_each_X label2];];
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
        
        this_round_train_Y=[this_round_train_Y; [new_this_round(each_slope+1,7:9) this_round_EMG_each_Y this_round_pass0_each_Y];];
        
    end
         %——看看EMG变化情况
    if this_round_train_X(1,end) == 5 || this_round_train_X(1,end) == 6
        figure(count_see);
        plot(this_round_train_X(:,7));  % up down 6 left right 7 for back 5
%         hold on;
%         plot(this_round_train_X(:,end)/10)
        count_see=count_see+1;
    end
    %% 
    if file < seperate_rate
        all_round_thisd_train_X=[all_round_thisd_train_X; this_round_train_X];
        all_round_thisd_train_Y=[all_round_thisd_train_Y; this_round_train_Y];
    else
        all_round_thisd_test_X=[all_round_thisd_test_X; this_round_train_X];
        all_round_thisd_test_Y=[all_round_thisd_test_Y; this_round_train_Y];  
    end
end



    all_round_train_X=[all_round_train_X; all_round_thisd_train_X];
    all_round_train_Y=[all_round_train_Y; all_round_thisd_train_Y];
    
    all_round_test_X=[all_round_test_X; all_round_thisd_test_X];
    all_round_test_Y=[all_round_test_Y; all_round_thisd_test_Y];    
%     pause(10);
end
all_round_train_X=all_round_train_X;
all_round_train_Y=all_round_train_Y;
all_round_test_X=all_round_test_X;
all_round_test_Y=all_round_test_Y;

% all_round_train_X=all_round_train_X(:,[1:8 end]);
% all_round_train_Y=all_round_train_Y(:,[1:8]);
% all_round_test_X=all_round_test_X(:,[1:8 end]);
% all_round_test_Y=all_round_test_Y(:,[1:8]);

%% Train INIT
X_train=all_round_train_X;
Y_train=all_round_train_Y;



end


















% figure(11) 
% for g = 1:4
%         expr_g=['which' num2str(g) ' = find(all_round_train_X(:,end)==' num2str(g) '); '];
%         eval(expr_g);
%         expr_g2=['xyz' num2str(g) ' = all_round_train_X(which' num2str(g) ',1:3);'];
%         eval(expr_g2);
%         expr_g2=['EMG456' num2str(g) ' = all_round_train_X(which' num2str(g) ',4:6);'];
%         eval(expr_g2);       
%         expr_g2=['EMG678' num2str(g) ' = all_round_train_X(which' num2str(g) ',6:8);'];
%         eval(expr_g2);       
%         expr_g2=['pass91011' num2str(g) ' = all_round_train_X(which' num2str(g) ',9:11);'];
%         eval(expr_g2);               
%         expr_g2=['pass111213' num2str(g) ' = all_round_train_X(which' num2str(g) ',11:13);'];
%         eval(expr_g2);               
%         
%         
%         expr_g3 = ['plot3(xyz' num2str(g) '(:,1), xyz' num2str(g) '(:,2), xyz' num2str(g) '(:,3), "o"); hold on;'];
%         eval(expr_g3);% xyz
% end
% 
% figure(12)
% for g = 1:4
%         expr_g3 = ['plot3(EMG456' num2str(g) '(:,1), EMG456' num2str(g) '(:,2), EMG456' num2str(g) '(:,3),"o"); hold on;'];
%         eval(expr_g3);
% end
% figure(13)
% for g = 1:4
%         expr_g3 = ['plot3(EMG678' num2str(g) '(:,1), EMG678' num2str(g) '(:,2), EMG678' num2str(g) '(:,3),"o"); hold on;'];
%         eval(expr_g3);
% end
% 
% figure(14)
% for g = 1:4
%         expr_g3 = ['plot3(pass91011' num2str(g) '(:,1), pass91011' num2str(g) '(:,2), pass91011' num2str(g) '(:,3),"o"); hold on;'];
%         eval(expr_g3);
% end
% 
% figure(15)
% for g = 1:4
%         expr_g3 = ['plot3(pass111213' num2str(g) '(:,1), pass111213' num2str(g) '(:,2), pass111213' num2str(g) '(:,3),"o"); hold on;'];
%         eval(expr_g3);
% end
% 
