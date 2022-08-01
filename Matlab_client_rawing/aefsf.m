close all;clear;clc;
% use GPR to predict
load('20220125good_down.mat')


% load('20220119fial.mat');

% roundfrom1=[all_end_effector_p(:,5000:end)]; 
% roundfrom2=[all_end_effector_p(:,5000:end)]; 

% roundfrom1=[all_end_effector_p(:,2200:4000)]; 
% roundfrom2=[all_end_effector_p(:,2200:4000)]; 




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


chuyi=all_end_effector_p(2,6000:8000)./tan(angle_comb);
neww=[chuyi; all_end_effector_p(2:end,6000:8000)];
roundfrom1=neww;
roundfrom2=neww;

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

all_round_test_X = delete_lessdata(all_round_test_X);




figure(90)
plot(all_round_test_X(:,end)); hold on;
plot(all_round_test_X(:,1))

processed_X=[];

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




