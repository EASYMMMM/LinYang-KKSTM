close all;clear;clc;
% this file is used to divide the raw series of data into labeled data.
% 2021-12-07

% % 左右上下后前——123456
%%
round1= load ('tableup1_20211215_zengcheng.mat').round1;  % good 
round2= load ('tableup2_20211215_zengcheng.mat').sychronize;  % good
round3= load ('tableup3_20211215_zengcheng.mat').sychronize; % fine 
round4= load ('tableup4_20211215_zengcheng.mat').sychronize; %MDZZ
round5= load ('tableback1_20211215_zengcheng.mat').sychronize; % good
round6= load ('tableback2_20211215_zengcheng.mat').sychronize;  % fine
round7= load ('tableback4_20211215_zengcheng.mat').sychronize;
round8= load ('tableback5_20211215_zengcheng.mat').sychronize;

for file = 8
    name_g=['round' num2str(file)];
    round_this=eval(name_g);
    
round_this = transform(round_this) ;
    xyz=[];
    
    
    
        for FLAG = 1:3
            if FLAG == 2 % y
                wrist_direction=8; lower_bound=40; upper_bound=900; label1=3; label2=4; howmany = 700;
            elseif FLAG == 3 % z
                wrist_direction=9; lower_bound=60; upper_bound=500; label1=5; label2=6; howmany = 700;
            elseif FLAG == 1  % x
                wrist_direction=7; lower_bound=85; upper_bound=750; label1=1; label2=2; howmany = 700;
            end
            t=1:length(round_this(:,wrist_direction));
            p=polyfit(t,round_this(:,wrist_direction),20);
            yi=polyval(p,t).';
            xyz=[xyz yi];
        end
        
%     figure(file+20); %——注释掉，调试用的
%     plot(round_this(:,7:9));
%     hold on;
%     plot(new_EMG2); 
%     legend('real-x', 'real-y', 'real-z', 'adjust-x','adjust-y','adjust-z')
%     legend('real-x', 'real-y', 'real-z', 'fit-x', 'fit-y', 'fit-z', 'adjust-x','adjust-y','adjust-z')
    
    sychronize=[round_this(:,1:6) xyz round_this(:,10:end)];
end
% save('tableback1_20211215_zengcheng_clear','sychronize')
