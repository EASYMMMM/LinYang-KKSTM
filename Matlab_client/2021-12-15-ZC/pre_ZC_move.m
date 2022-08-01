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
round1= load ('tableup1_20211215_zengcheng.mat').round1;
round2= load ('tableup2_20211215_zengcheng.mat').sychronize;  % ???
round3= load ('tableup3_20211215_zengcheng.mat').sychronize;
round4= load ('tableup4_20211215_zengcheng.mat').sychronize;
round5= load ('tableback1_20211215_zengcheng.mat').sychronize;
round6= load ('tableback2_20211215_zengcheng.mat').sychronize;  % ???
round7= load ('tableback4_20211215_zengcheng.mat').sychronize;
round8= load ('tableback5_20211215_zengcheng.mat').sychronize;


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

seperate_rate=2;
%%
all_round_train_X=[]; all_round_train_Y=[];all_round_test_Y=[];all_round_test_X=[];
for FLAG = 1:3
    if FLAG == 2 % y
        wrist_direction=8; lower_bound=40; upper_bound=900; label1=3; label2=4; howmany = 700;
    elseif FLAG == 3 % z
        wrist_direction=9; lower_bound=60; upper_bound=500; label1=5; label2=6; howmany = 700;
    elseif FLAG == 1  % x
        wrist_direction=7; lower_bound=85; upper_bound=750; label1=1; label2=2; howmany = 700;
    end
    
    
%%  
all_round_thisd_train_X=[]; all_round_thisd_train_Y=[];all_round_thisd_test_X=[]; all_round_thisd_test_Y=[];
for file = 1:4
    % 不必要的
    if file <=4 && FLAG == 1
        continue
    end
    
    if file >= 5 && FLAG == 2
        continue
    end

    
    
    
    
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
    if FLAG == 3
        which_want=find(slope > threhold);
    else
        which_want=find(abs(slope) > threhold);
    end
    new_which_want=[];
    for z = 1:length(which_want)
        oo=which_want(z);
        if oo < upper_bound && oo > lower_bound
            new_which_want=[new_which_want oo];
        end
    end

    want=yi(new_which_want);

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