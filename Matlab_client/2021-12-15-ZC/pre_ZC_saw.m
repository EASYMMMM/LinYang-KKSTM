close all;clear;clc;
% this file is used to divide the raw series of data into labeled data.
% 2021-12-18

% % 左右上下后前——123456
%%
round1= load ('saw1_20211215_zengcheng.mat').sychronize;
round2= load ('saw2_20211215_zengcheng.mat').sychronize;  % ???
round3= load ('saw3_20211215_zengcheng.mat').sychronize;
round4= load ('saw4_20211215_zengcheng.mat').sychronize;
round5= load ('saw5_20211215_zengcheng.mat').sychronize;
round6= load ('saw6_20211215_zengcheng.mat').sychronize;

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
seperate_rate=2;order=10;
list_seperate_rate=2:1:2; list_rate_add=0:2;see_slope=0;
ALL_MSE=[];ALL_result=[];ALL_result_use=[];

%%
all_round_train_X=[]; all_round_train_Y=[];all_round_test_Y=[];all_round_test_X=[];
for FLAG = 1:3
    if FLAG == 2 % y
        wrist_direction=8; lower_bound=298; upper_bound=769; label1=3; label2=4; howmany = 800;
    elseif FLAG == 3 % z
        wrist_direction=9; lower_bound=60; upper_bound=326; label1=5; label2=6; howmany = 800;
    elseif FLAG == 1  % x
        wrist_direction=7; lower_bound1=10; upper_bound1=563; label1=1; label2=2; howmany = 800;
         lower_bound2=10; upper_bound2=563;
    end
    
    
%%  
all_round_thisd_train_X=[]; all_round_thisd_train_Y=[];all_round_thisd_test_X=[]; all_round_thisd_test_Y=[];
for file = 1:1

    
    name_g=['round' num2str(file)];
    round_this=eval(name_g);
    
    
    %%
    if FLAG == 1
        
    t1=1:length(round_this(lower_bound1:upper_bound1,wrist_direction));
    p1=polyfit(t1,round_this(lower_bound1:upper_bound1,wrist_direction),60);
    yi1=polyval(p1,t1);
    t2=1:length(round_this(lower_bound2:upper_bound2,wrist_direction));
    p2=polyfit(t,round_this(lower_bound2:upper_bound2,wrist_direction),60);
    yi2=polyval(p,t);
    
    figure(65)
    plot(t1,round_this(lower_bound1:upper_bound2,wrist_direction),':o',t1,yi1,'-*'); hold on; %——注释掉，调试用的
    figure(66)
    plot(t2,round_this(lower_bound2:upper_bound2,wrist_direction),':o',t2,yi2,'-*'); hold on; %——注释掉，调试用的    
    
    yi=[yi; y2;];
    end
    %     Stepnumber=lower_bound:upper_bound;
%     Grayvalue=round_this(lower_bound:upper_bound,wrist_direction);
%     func_sin = @(a,t) a(1)*sin(a(4)*t+a(2)) + a(3);
% 
%     A= lsqcurvefit( func_sin, [1 0 1.3 pi/10], Stepnumber.', Grayvalue);
%     figure(55);
%     plot(Stepnumber, Grayvalue, 'r*')
%     hold on
%     plot(Stepnumber, func_sin(A,Stepnumber))
    
    
    
    
    
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
    

    
    
%         if FLAG == 1
%             count=0;simple_want=[];temp_want=[];
%             inv_new_which_want=fliplr(new_which_want);
%             for k = 1:length(inv_new_which_want)-1
%                 this_want=inv_new_which_want(k);
%                 next_want=inv_new_which_want(k+1);
%                 if this_want-1 == next_want && count < 2 
%                     temp_want=[temp_want this_want];
% 
%                 else
%                     if length(temp_want) > 20
%                         simple_want=[simple_want temp_want];
%                         count=count+1;
%                     end 
%                     temp_want=[];
% 
%                 end
%             end
%         new_which_want=fliplr(simple_want);
%         want=yi(new_which_want);       
% 
%         end


    
     %——注释掉，调试用的
    figure(4+file);
    plot(t,round_this(:,wrist_direction),':o',t,yi,'-*'); hold on;
    plot(new_which_want,want,'o','MarkerSize',10)
    
    
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
    px=polyfit(t,round_this(:,7),order);
    py=polyfit(t,round_this(:,8),order);
    pz=polyfit(t,round_this(:,9),order);
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