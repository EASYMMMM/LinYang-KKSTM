close all;clear;clc;
warning('off')
% To Lin YANG其实说白了，需要关注的只有主轴运动，z轴运动，还有EMG，还有啥啊？
%1 is YANG Lin; 2 is wanjiancheng; 3 is liuhanjie; 4 is hunnaxing, 5 is
%liuxiaodong, 6 is zengshaofeng zls, 7 is zengcheng, 8 is qiqianshuo
% 9 is zhaoyining 10 i sbaorunyuan
allresults=[];
for name = 6:10

%%
if name == 1

M0=load('20220317_LY_good_M0.mat');
M1=load('20220317_LY_good_M1.mat');
M2=load('20220317_LY_good_M2.mat');
M3=load('20220317_LY_fine_M3.mat');

M1_result=M1.result;
M2_result=M2.result;
M3_result=M3.result;
%% z轴深度M0
start_M0=1392; end_M0=12974; 
M0_all_end_effector_p_z=M0.all_end_effector_p(3,start_M0:end_M0);
avg_z0=(M0_all_end_effector_p_z(1)-M0_all_end_effector_p_z(end))/size(M0_all_end_effector_p_z,2);
M0_all_EMG=M0.all_EMG(2:end-1,start_M0:end_M0);
avg_EMG_M0=sum(M0_all_EMG,2)/size(M0_all_EMG,2);

vx_M0=M0.all_end_effector_p(1,start_M0+1:end_M0+1)-M0.all_end_effector_p(1,start_M0-1:end_M0-1);
vz_M0=M0.all_end_effector_p(3,start_M0+1:end_M0+1)-M0.all_end_effector_p(3,start_M0-1:end_M0-1);
vx_M0=abs(vx_M0);
vz_M0=abs(vz_M0);

each_EMG_div_vz_vx_M0=sum(M0_all_EMG,1)./sum(vx_M0,1)./sum(vz_M0,1); % the power for each point
avg_each_EMG_div_vz_vx_M0=mean(each_EMG_div_vz_vx_M0);
std0_forv_div_vx=(1/length(each_EMG_div_vz_vx_M0)*sum((each_EMG_div_vz_vx_M0-mean(each_EMG_div_vz_vx_M0)).^2))^0.5;

each_EMG_div_vz_M0=sum(M0_all_EMG,1)./sum(vz_M0,1); % the power for each point
std0_forzv=(1/length(each_EMG_div_vz_M0)*sum((each_EMG_div_vz_M0-mean(each_EMG_div_vz_M0)).^2))^0.5;

%% z轴深度M1
start_M1=4384; end_M1=10335; 
M1_all_end_effector_p_z=M1.all_end_effector_p(3,start_M1:end_M1);
avg_z1=(M1_all_end_effector_p_z(1)-M1_all_end_effector_p_z(end))/size(M1_all_end_effector_p_z,2);
M1_all_EMG=M1.all_EMG(2:end-1,start_M1:end_M1);
avg_EMG_M1=sum(M1_all_EMG,2)/size(M1_all_EMG,2);

vz_M1=M1.all_end_effector_p(3,start_M1+1:end_M1+1)-M1.all_end_effector_p(3,start_M1-1:end_M1-1);
vx_M1=M1.all_end_effector_p(1,start_M1+1:end_M1+1)-M1.all_end_effector_p(1,start_M1-1:end_M1-1);
vx_M1=abs(vx_M1);
vz_M1=abs(vz_M1);
each_EMG_div_vxz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1)./sum(vx_M1,1); % the power for each point
avg_each_EMG_div_vz_vx_M1=mean(each_EMG_div_vxz_M1);
std1_forv_div_vx=(1/length(each_EMG_div_vxz_M1)*sum((each_EMG_div_vxz_M1-mean(each_EMG_div_vxz_M1)).^2))^0.5;

each_EMG_div_vz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1); % the power for each point
std1_forzv=(1/length(each_EMG_div_vz_M1)*sum((each_EMG_div_vz_M1-mean(each_EMG_div_vz_M1)).^2))^0.5;


%M2
start_state_M2_1=2812; start_state_M2_2=11448; 
% how_many_del1=length(find(M2_result(1:start_state_M2_1,1) == 4));
% how_many_del2=length(find(M2_result(1:start_state_M2_2,1) == 4));

M2_all_end_effector_p_z=M2.all_end_effector_p(3,start_state_M2_1:start_state_M2_2);

avg_z2=(M2_all_end_effector_p_z(1)-M2_all_end_effector_p_z(end))/size(M2_all_end_effector_p_z,2);

M2_all_EMG=M2.all_EMG(2:end-1,start_state_M2_1:start_state_M2_2);
avg_EMG_M2=sum(M2_all_EMG,2)/size(M2_all_EMG,2);

vx_M2=M2.all_end_effector_p(1,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(1,start_state_M2_1-1:start_state_M2_2-1);
vz_M2=M2.all_end_effector_p(3,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(3,start_state_M2_1-1:start_state_M2_2-1);
vx_M2=abs(vx_M2);
vz_M2=abs(vz_M2);

each_EMG_div_vxz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1)./sum(vx_M2,1); % the power for each point
avg_each_EMG_div_vz_vx_M2=mean(each_EMG_div_vxz_M2);
std2_forv_div_vx=(1/length(each_EMG_div_vxz_M2)*sum((each_EMG_div_vxz_M2-mean(each_EMG_div_vxz_M2)).^2))^0.5;

each_EMG_div_vz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1); % the power for each point
std2_forzv=(1/length(each_EMG_div_vz_M2)*sum((each_EMG_div_vz_M2-mean(each_EMG_div_vz_M2)).^2))^0.5;




% M3
start_state_M3_1=3839; start_state_M3_2=7467; 
% how_many_del1=length(find(M3_result(1:start_state_M3_1,1) == 4));
% how_many_del2=length(find(M3_result(1:start_state_M3_2,1) == 4));
M3_all_end_effector_p_z=M3.all_end_effector_p(3,start_state_M3_1:start_state_M3_2);
avg_z3=(M3_all_end_effector_p_z(1)-M3_all_end_effector_p_z(end))/size(M3_all_end_effector_p_z,2);

M3_all_EMG=M3.all_EMG(2:end-1,start_state_M3_1:start_state_M3_2);
avg_EMG_M3=sum(M3_all_EMG,2)/size(M3_all_EMG,2);

vx_M3=M3.all_end_effector_p(1,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(1,start_state_M3_1-1:start_state_M3_2-1);
vz_M3=M3.all_end_effector_p(3,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(3,start_state_M3_1-1:start_state_M3_2-1);
vx_M3=abs(vx_M3);
vz_M3=abs(vz_M3);

each_EMG_div_vxz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1)./sum(vx_M3,1); % the power for each point
avg_each_EMG_div_vz_vx_M3=mean(each_EMG_div_vxz_M3);
std3_forv_div_vx=(1/length(each_EMG_div_vxz_M3)*sum((each_EMG_div_vxz_M3-mean(each_EMG_div_vxz_M3)).^2))^0.5;

each_EMG_div_vz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1); % the power for each point
std3_forzv=(1/length(each_EMG_div_vz_M3)*sum((each_EMG_div_vz_M3-mean(each_EMG_div_vz_M3)).^2))^0.5;


sum_avg_EMG_M0=sum(avg_EMG_M0);
sum_avg_EMG_M1=sum(avg_EMG_M1);
sum_avg_EMG_M2=sum(avg_EMG_M2);
sum_avg_EMG_M3=sum(avg_EMG_M3);
avg_z0;
avg_z1;
avg_z2;
avg_z3;


power_div_z0=sum_avg_EMG_M0/avg_z0
power_div_z1=sum_avg_EMG_M1/avg_z1
power_div_z2=sum_avg_EMG_M2/avg_z2
power_div_z3=sum_avg_EMG_M3/avg_z3

std0_forv_div_vx
std1_forv_div_vx
std2_forv_div_vx
std3_forv_div_vx

std0_forzv
std1_forzv
std2_forzv
std3_forzv

avg_each_EMG_div_vz_vx_M0
avg_each_EMG_div_vz_vx_M1
avg_each_EMG_div_vz_vx_M2
avg_each_EMG_div_vz_vx_M3

%% 分档
all_M3z=M3.all_end_effector_p;
low_frequency1=3760;low_frequency2=5184;
how_many_del1=length(find(M2_result(1:low_frequency1,1) == 4));
how_many_del2=length(find(M2_result(1:low_frequency2,1) == 4));
M3_low_z=M3.all_end_effector_p(3,low_frequency1-how_many_del1:low_frequency2-how_many_del2);
avg_low_z3=(M3_low_z(1)-M3_low_z(end))/size(M3_low_z,2);
M3_low_EMG=M3.all_EMG(2:end-1,low_frequency1-how_many_del1:low_frequency2-how_many_del2);
avg_low_EMG_M3=sum(M3_low_EMG,2)/size(M3_low_EMG,2);
sum(avg_low_EMG_M3);
end

%======================================
% name=2

if name == 2

M3=load('20220317_wjc_fine_M3.mat');
M1=load('20220317_wjc_fine_M1.mat');
M2=load('20220317_wjc_good_M2.mat');
M0=load('20220317_wjc_fine_M0.mat'); % 翘起来了。。。

M1_result=M1.result;
M2_result=M2.result;
M3_result=M3.result;

% EMG/(m/s)
%% z轴深度M0

start_M0=1117; end_M0=1512; 
M0_all_end_effector_p_z=M0.all_end_effector_p(3,start_M0:end_M0);
avg_z0=(M0_all_end_effector_p_z(1)-M0_all_end_effector_p_z(end))/size(M0_all_end_effector_p_z,2);
M0_all_EMG=M0.all_EMG(2:end-1,start_M0:end_M0);
avg_EMG_M0=sum(M0_all_EMG,2)/size(M0_all_EMG,2);

vx_M0=M0.all_end_effector_p(1,start_M0+1:end_M0+1)-M0.all_end_effector_p(1,start_M0-1:end_M0-1);
vz_M0=M0.all_end_effector_p(3,start_M0+1:end_M0+1)-M0.all_end_effector_p(3,start_M0-1:end_M0-1);
vx_M0=abs(vx_M0);
vz_M0=abs(vz_M0);

each_EMG_div_vz_vx_M0=sum(M0_all_EMG,1)./sum(vx_M0,1)./sum(vz_M0,1); % the power for each point
avg_each_EMG_div_vz_vx_M0=mean(each_EMG_div_vz_vx_M0);
std0_forv_div_vx=(1/length(each_EMG_div_vz_vx_M0)*sum((each_EMG_div_vz_vx_M0-mean(each_EMG_div_vz_vx_M0)).^2))^0.5;

each_EMG_div_vz_M0=sum(M0_all_EMG,1)./sum(vz_M0,1); % the power for each point
std0_forzv=(1/length(each_EMG_div_vz_M0)*sum((each_EMG_div_vz_M0-mean(each_EMG_div_vz_M0)).^2))^0.5;

%% z轴深度M1
start_M1=1150; end_M1=4718; 
M1_all_end_effector_p_z=M1.all_end_effector_p(3,start_M1:end_M1);
avg_z1=(M1_all_end_effector_p_z(1)-M1_all_end_effector_p_z(end))/size(M1_all_end_effector_p_z,2);
M1_all_EMG=M1.all_EMG(2:end-1,start_M1:end_M1);
avg_EMG_M1=sum(M1_all_EMG,2)/size(M1_all_EMG,2);

vz_M1=M1.all_end_effector_p(3,start_M1+1:end_M1+1)-M1.all_end_effector_p(3,start_M1-1:end_M1-1);
vx_M1=M1.all_end_effector_p(1,start_M1+1:end_M1+1)-M1.all_end_effector_p(1,start_M1-1:end_M1-1);
vx_M1=abs(vx_M1);
vz_M1=abs(vz_M1);
each_EMG_div_vxz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1)./sum(vx_M1,1); % the power for each point
avg_each_EMG_div_vz_vx_M1=mean(each_EMG_div_vxz_M1);
std1_forv_div_vx=(1/length(each_EMG_div_vxz_M1)*sum((each_EMG_div_vxz_M1-mean(each_EMG_div_vxz_M1)).^2))^0.5;

each_EMG_div_vz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1); % the power for each point
std1_forzv=(1/length(each_EMG_div_vz_M1)*sum((each_EMG_div_vz_M1-mean(each_EMG_div_vz_M1)).^2))^0.5;


%M2
start_state_M2_1=4593; start_state_M2_2=7775; 
M2_all_end_effector_p_z=M2.all_end_effector_p(3,start_state_M2_1:start_state_M2_2);
avg_z2=(M2_all_end_effector_p_z(1)-M2_all_end_effector_p_z(end))/size(M2_all_end_effector_p_z,2);

M2_all_EMG=M2.all_EMG(2:end-1,start_state_M2_1:start_state_M2_2);
avg_EMG_M2=sum(M2_all_EMG,2)/size(M2_all_EMG,2);

vx_M2=M2.all_end_effector_p(1,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(1,start_state_M2_1-1:start_state_M2_2-1);
vz_M2=M2.all_end_effector_p(3,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(3,start_state_M2_1-1:start_state_M2_2-1);
vx_M2=abs(vx_M2);
vz_M2=abs(vz_M2);

each_EMG_div_vxz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1)./sum(vx_M2,1); % the power for each point
avg_each_EMG_div_vz_vx_M2=mean(each_EMG_div_vxz_M2);
std2_forv_div_vx=(1/length(each_EMG_div_vxz_M2)*sum((each_EMG_div_vxz_M2-mean(each_EMG_div_vxz_M2)).^2))^0.5;

each_EMG_div_vz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1); % the power for each point
std2_forzv=(1/length(each_EMG_div_vz_M2)*sum((each_EMG_div_vz_M2-mean(each_EMG_div_vz_M2)).^2))^0.5;




% M3
start_state_M3_1=3014; start_state_M3_2=5331; 
M3_all_end_effector_p_z=M3.all_end_effector_p(3,start_state_M3_1:start_state_M3_2);
avg_z3=(M3_all_end_effector_p_z(1)-M3_all_end_effector_p_z(end))/size(M3_all_end_effector_p_z,2);

M3_all_EMG=M3.all_EMG(2:end-1,start_state_M3_1:start_state_M3_2);
avg_EMG_M3=sum(M3_all_EMG,2)/size(M3_all_EMG,2);

vx_M3=M3.all_end_effector_p(1,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(1,start_state_M3_1-1:start_state_M3_2-1);
vz_M3=M3.all_end_effector_p(3,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(3,start_state_M3_1-1:start_state_M3_2-1);
vx_M3=abs(vx_M3);
vz_M3=abs(vz_M3);

each_EMG_div_vxz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1)./sum(vx_M3,1); % the power for each point
avg_each_EMG_div_vz_vx_M3=mean(each_EMG_div_vxz_M3);
std3_forv_div_vx=(1/length(each_EMG_div_vxz_M3)*sum((each_EMG_div_vxz_M3-mean(each_EMG_div_vxz_M3)).^2))^0.5;

each_EMG_div_vz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1); % the power for each point
std3_forzv=(1/length(each_EMG_div_vz_M3)*sum((each_EMG_div_vz_M3-mean(each_EMG_div_vz_M3)).^2))^0.5;


sum_avg_EMG_M0=sum(avg_EMG_M0);
sum_avg_EMG_M1=sum(avg_EMG_M1);
sum_avg_EMG_M2=sum(avg_EMG_M2);
sum_avg_EMG_M3=sum(avg_EMG_M3);
avg_z0;
avg_z1;
avg_z2;
avg_z3;


power_div_z0=sum_avg_EMG_M0/avg_z0
power_div_z1=sum_avg_EMG_M1/avg_z1
power_div_z2=sum_avg_EMG_M2/avg_z2
power_div_z3=sum_avg_EMG_M3/avg_z3

std0_forv_div_vx
std1_forv_div_vx
std2_forv_div_vx
std3_forv_div_vx

std0_forzv
std1_forzv
std2_forzv
std3_forzv

avg_each_EMG_div_vz_vx_M0
avg_each_EMG_div_vz_vx_M1
avg_each_EMG_div_vz_vx_M2
avg_each_EMG_div_vz_vx_M3
%% 分档
all_M3z=M3.all_end_effector_p;
low_frequency1=3760;low_frequency2=5184;
how_many_del1=length(find(M2_result(1:low_frequency1,1) == 4));
how_many_del2=length(find(M2_result(1:low_frequency2,1) == 4));
M3_low_z=M3.all_end_effector_p(3,low_frequency1-how_many_del1:low_frequency2-how_many_del2);
avg_low_z3=(M3_low_z(1)-M3_low_z(end))/size(M3_low_z,2);
M3_low_EMG=M3.all_EMG(2:end-1,low_frequency1-how_many_del1:low_frequency2-how_many_del2);
avg_low_EMG_M3=sum(M3_low_EMG,2)/size(M3_low_EMG,2);
sum(avg_low_EMG_M3);
end


if name == 3

M3=load('20220324_lhj_M3_fine.mat');
M1=load('20220324_lhj_M1_fine.mat');
M2=load('20220324_lhj_M2_godd.mat');
M0=load('20220324_lhj_M0_good.mat'); % 翘起来了。。。

M1_result=M1.result;
M2_result=M2.result;
M3_result=M3.result;

% EMG/(m/s)
%% z轴深度M0

start_M0=1412; end_M0=5672; 
M0_all_end_effector_p_z=M0.all_end_effector_p(3,start_M0:end_M0);
avg_z0=(M0_all_end_effector_p_z(1)-M0_all_end_effector_p_z(end))/size(M0_all_end_effector_p_z,2);
M0_all_EMG=M0.all_EMG(2:end-1,start_M0:end_M0);
avg_EMG_M0=sum(M0_all_EMG,2)/size(M0_all_EMG,2);

vx_M0=M0.all_end_effector_p(1,start_M0+1:end_M0+1)-M0.all_end_effector_p(1,start_M0-1:end_M0-1);
vz_M0=M0.all_end_effector_p(3,start_M0+1:end_M0+1)-M0.all_end_effector_p(3,start_M0-1:end_M0-1);
vx_M0=abs(vx_M0);
vz_M0=abs(vz_M0);

each_EMG_div_vz_vx_M0=sum(M0_all_EMG,1)./sum(vx_M0,1)./sum(vz_M0,1); % the power for each point
avg_each_EMG_div_vz_vx_M0=mean(each_EMG_div_vz_vx_M0);
std0_forv_div_vx=(1/length(each_EMG_div_vz_vx_M0)*sum((each_EMG_div_vz_vx_M0-mean(each_EMG_div_vz_vx_M0)).^2))^0.5;

each_EMG_div_vz_M0=sum(M0_all_EMG,1)./sum(vz_M0,1); % the power for each point
std0_forzv=(1/length(each_EMG_div_vz_M0)*sum((each_EMG_div_vz_M0-mean(each_EMG_div_vz_M0)).^2))^0.5;

%% z轴深度M1
start_M1=601; end_M1=2523; 
M1_all_end_effector_p_z=M1.all_end_effector_p(3,start_M1:end_M1);
avg_z1=(M1_all_end_effector_p_z(1)-M1_all_end_effector_p_z(end))/size(M1_all_end_effector_p_z,2);
M1_all_EMG=M1.all_EMG(2:end-1,start_M1:end_M1);
avg_EMG_M1=sum(M1_all_EMG,2)/size(M1_all_EMG,2);

vz_M1=M1.all_end_effector_p(3,start_M1+1:end_M1+1)-M1.all_end_effector_p(3,start_M1-1:end_M1-1);
vx_M1=M1.all_end_effector_p(1,start_M1+1:end_M1+1)-M1.all_end_effector_p(1,start_M1-1:end_M1-1);
vx_M1=abs(vx_M1);
vz_M1=abs(vz_M1);
each_EMG_div_vxz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1)./sum(vx_M1,1); % the power for each point
avg_each_EMG_div_vz_vx_M1=mean(each_EMG_div_vxz_M1);
std1_forv_div_vx=(1/length(each_EMG_div_vxz_M1)*sum((each_EMG_div_vxz_M1-mean(each_EMG_div_vxz_M1)).^2))^0.5;

each_EMG_div_vz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1); % the power for each point
std1_forzv=(1/length(each_EMG_div_vz_M1)*sum((each_EMG_div_vz_M1-mean(each_EMG_div_vz_M1)).^2))^0.5;


%M2
start_state_M2_1=1319; start_state_M2_2=4949; 
M2_all_end_effector_p_z=M2.all_end_effector_p(3,start_state_M2_1:start_state_M2_2);
avg_z2=(M2_all_end_effector_p_z(1)-M2_all_end_effector_p_z(end))/size(M2_all_end_effector_p_z,2);

M2_all_EMG=M2.all_EMG(2:end-1,start_state_M2_1:start_state_M2_2);
avg_EMG_M2=sum(M2_all_EMG,2)/size(M2_all_EMG,2);

vx_M2=M2.all_end_effector_p(1,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(1,start_state_M2_1-1:start_state_M2_2-1);
vz_M2=M2.all_end_effector_p(3,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(3,start_state_M2_1-1:start_state_M2_2-1);
vx_M2=abs(vx_M2);
vz_M2=abs(vz_M2);

each_EMG_div_vxz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1)./sum(vx_M2,1); % the power for each point
avg_each_EMG_div_vz_vx_M2=mean(each_EMG_div_vxz_M2);
std2_forv_div_vx=(1/length(each_EMG_div_vxz_M2)*sum((each_EMG_div_vxz_M2-mean(each_EMG_div_vxz_M2)).^2))^0.5;

each_EMG_div_vz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1); % the power for each point
std2_forzv=(1/length(each_EMG_div_vz_M2)*sum((each_EMG_div_vz_M2-mean(each_EMG_div_vz_M2)).^2))^0.5;




% M3
start_state_M3_1=1013; start_state_M3_2=1993; 
M3_all_end_effector_p_z=M3.all_end_effector_p(3,start_state_M3_1:start_state_M3_2);
avg_z3=(M3_all_end_effector_p_z(1)-M3_all_end_effector_p_z(end))/size(M3_all_end_effector_p_z,2);

M3_all_EMG=M3.all_EMG(2:end-1,start_state_M3_1:start_state_M3_2);
avg_EMG_M3=sum(M3_all_EMG,2)/size(M3_all_EMG,2);

vx_M3=M3.all_end_effector_p(1,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(1,start_state_M3_1-1:start_state_M3_2-1);
vz_M3=M3.all_end_effector_p(3,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(3,start_state_M3_1-1:start_state_M3_2-1);
vx_M3=abs(vx_M3);
vz_M3=abs(vz_M3);

each_EMG_div_vxz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1)./sum(vx_M3,1); % the power for each point
avg_each_EMG_div_vz_vx_M3=mean(each_EMG_div_vxz_M3);
std3_forv_div_vx=(1/length(each_EMG_div_vxz_M3)*sum((each_EMG_div_vxz_M3-mean(each_EMG_div_vxz_M3)).^2))^0.5;

each_EMG_div_vz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1); % the power for each point
std3_forzv=(1/length(each_EMG_div_vz_M3)*sum((each_EMG_div_vz_M3-mean(each_EMG_div_vz_M3)).^2))^0.5;


sum_avg_EMG_M0=sum(avg_EMG_M0);
sum_avg_EMG_M1=sum(avg_EMG_M1);
sum_avg_EMG_M2=sum(avg_EMG_M2);
sum_avg_EMG_M3=sum(avg_EMG_M3);
avg_z0;
avg_z1;
avg_z2;
avg_z3;


power_div_z0=sum_avg_EMG_M0/avg_z0
power_div_z1=sum_avg_EMG_M1/avg_z1
power_div_z2=sum_avg_EMG_M2/avg_z2
power_div_z3=sum_avg_EMG_M3/avg_z3

std0_forv_div_vx
std1_forv_div_vx
std2_forv_div_vx
std3_forv_div_vx

std0_forzv
std1_forzv
std2_forzv
std3_forzv

avg_each_EMG_div_vz_vx_M0
avg_each_EMG_div_vz_vx_M1
avg_each_EMG_div_vz_vx_M2
avg_each_EMG_div_vz_vx_M3
end
if name == 4
M0=load('20220329hnxM0_good.mat'); % 翘起来了。。。
M1=load('20220329hnx_M1_fine.mat');
M2=load('20220329hnx_M2_good.mat');
M3=load('20220329hnx_M3_good.mat');

M1_result=M1.result;
M2_result=M2.result;
M3_result=M3.result;

% EMG/(m/s)
%% z轴深度M0

start_M0=1032; end_M0=5575; 
M0_all_end_effector_p_z=M0.all_end_effector_p(3,start_M0:end_M0);
avg_z0=(M0_all_end_effector_p_z(1)-M0_all_end_effector_p_z(end))/size(M0_all_end_effector_p_z,2);
M0_all_EMG=M0.all_EMG(2:end-1,start_M0:end_M0);
avg_EMG_M0=sum(M0_all_EMG,2)/size(M0_all_EMG,2);

vx_M0=M0.all_end_effector_p(1,start_M0+1:end_M0+1)-M0.all_end_effector_p(1,start_M0-1:end_M0-1);
vz_M0=M0.all_end_effector_p(3,start_M0+1:end_M0+1)-M0.all_end_effector_p(3,start_M0-1:end_M0-1);
vx_M0=abs(vx_M0);
vz_M0=abs(vz_M0);

each_EMG_div_vz_vx_M0=sum(M0_all_EMG,1)./sum(vx_M0,1)./sum(vz_M0,1); % the power for each point
avg_each_EMG_div_vz_vx_M0=mean(each_EMG_div_vz_vx_M0);
std0_forv_div_vx=(1/length(each_EMG_div_vz_vx_M0)*sum((each_EMG_div_vz_vx_M0-mean(each_EMG_div_vz_vx_M0)).^2))^0.5;

each_EMG_div_vz_M0=sum(M0_all_EMG,1)./sum(vz_M0,1); % the power for each point
std0_forzv=(1/length(each_EMG_div_vz_M0)*sum((each_EMG_div_vz_M0-mean(each_EMG_div_vz_M0)).^2))^0.5;

%% z轴深度M1
start_M1=5258; end_M1=8123; 
M1_all_end_effector_p_z=M1.all_end_effector_p(3,start_M1:end_M1);
avg_z1=(M1_all_end_effector_p_z(1)-M1_all_end_effector_p_z(end))/size(M1_all_end_effector_p_z,2);
M1_all_EMG=M1.all_EMG(2:end-1,start_M1:end_M1);
avg_EMG_M1=sum(M1_all_EMG,2)/size(M1_all_EMG,2);

vz_M1=M1.all_end_effector_p(3,start_M1+1:end_M1+1)-M1.all_end_effector_p(3,start_M1-1:end_M1-1);
vx_M1=M1.all_end_effector_p(1,start_M1+1:end_M1+1)-M1.all_end_effector_p(1,start_M1-1:end_M1-1);
vx_M1=abs(vx_M1);
vz_M1=abs(vz_M1);
each_EMG_div_vxz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1)./sum(vx_M1,1); % the power for each point
avg_each_EMG_div_vz_vx_M1=mean(each_EMG_div_vxz_M1);
std1_forv_div_vx=(1/length(each_EMG_div_vxz_M1)*sum((each_EMG_div_vxz_M1-mean(each_EMG_div_vxz_M1)).^2))^0.5;

each_EMG_div_vz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1); % the power for each point
std1_forzv=(1/length(each_EMG_div_vz_M1)*sum((each_EMG_div_vz_M1-mean(each_EMG_div_vz_M1)).^2))^0.5;


%M2
start_state_M2_1=3295; start_state_M2_2=5363; 
M2_all_end_effector_p_z=M2.all_end_effector_p(3,start_state_M2_1:start_state_M2_2);
avg_z2=(M2_all_end_effector_p_z(1)-M2_all_end_effector_p_z(end))/size(M2_all_end_effector_p_z,2);

M2_all_EMG=M2.all_EMG(2:end-1,start_state_M2_1:start_state_M2_2);
avg_EMG_M2=sum(M2_all_EMG,2)/size(M2_all_EMG,2);

vx_M2=M2.all_end_effector_p(1,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(1,start_state_M2_1-1:start_state_M2_2-1);
vz_M2=M2.all_end_effector_p(3,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(3,start_state_M2_1-1:start_state_M2_2-1);
vx_M2=abs(vx_M2);
vz_M2=abs(vz_M2);

each_EMG_div_vxz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1)./sum(vx_M2,1); % the power for each point
avg_each_EMG_div_vz_vx_M2=mean(each_EMG_div_vxz_M2);
std2_forv_div_vx=(1/length(each_EMG_div_vxz_M2)*sum((each_EMG_div_vxz_M2-mean(each_EMG_div_vxz_M2)).^2))^0.5;

each_EMG_div_vz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1); % the power for each point
std2_forzv=(1/length(each_EMG_div_vz_M2)*sum((each_EMG_div_vz_M2-mean(each_EMG_div_vz_M2)).^2))^0.5;




% M3
start_state_M3_1=4389; start_state_M3_2=4544; 
M3_all_end_effector_p_z=M3.all_end_effector_p(3,start_state_M3_1:start_state_M3_2);
avg_z3=(M3_all_end_effector_p_z(1)-M3_all_end_effector_p_z(end))/size(M3_all_end_effector_p_z,2);

M3_all_EMG=M3.all_EMG(2:end-1,start_state_M3_1:start_state_M3_2);
avg_EMG_M3=sum(M3_all_EMG,2)/size(M3_all_EMG,2);

vx_M3=M3.all_end_effector_p(1,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(1,start_state_M3_1-1:start_state_M3_2-1);
vz_M3=M3.all_end_effector_p(3,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(3,start_state_M3_1-1:start_state_M3_2-1);
vx_M3=abs(vx_M3);
vz_M3=abs(vz_M3);

each_EMG_div_vxz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1)./sum(vx_M3,1); % the power for each point
avg_each_EMG_div_vz_vx_M3=mean(each_EMG_div_vxz_M3);
std3_forv_div_vx=(1/length(each_EMG_div_vxz_M3)*sum((each_EMG_div_vxz_M3-mean(each_EMG_div_vxz_M3)).^2))^0.5;

each_EMG_div_vz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1); % the power for each point
std3_forzv=(1/length(each_EMG_div_vz_M3)*sum((each_EMG_div_vz_M3-mean(each_EMG_div_vz_M3)).^2))^0.5;


sum_avg_EMG_M0=sum(avg_EMG_M0);
sum_avg_EMG_M1=sum(avg_EMG_M1);
sum_avg_EMG_M2=sum(avg_EMG_M2);
sum_avg_EMG_M3=sum(avg_EMG_M3);
avg_z0;
avg_z1;
avg_z2;
avg_z3;


power_div_z0=sum_avg_EMG_M0/avg_z0
power_div_z1=sum_avg_EMG_M1/avg_z1
power_div_z2=sum_avg_EMG_M2/avg_z2
power_div_z3=sum_avg_EMG_M3/avg_z3

std0_forv_div_vx
std1_forv_div_vx
std2_forv_div_vx
std3_forv_div_vx

std0_forzv
std1_forzv
std2_forzv
std3_forzv

avg_each_EMG_div_vz_vx_M0
avg_each_EMG_div_vz_vx_M1
avg_each_EMG_div_vz_vx_M2
avg_each_EMG_div_vz_vx_M3
end
if name == 5

M0=load('20220404lxd_M0_good.mat'); % 翘起来了。。。
M1=load('20220404lxd_M1_good.mat');
M2=load('20220404lxd_M2_good.mat');
M3=load('20220404lxd_M3_good.mat');

M1_result=M1.result;
M2_result=M2.result;
M3_result=M3.result;

% EMG/(m/s)
%% z轴深度M0

start_M0=10614; end_M0=14734; 
M0_all_end_effector_p_z=M0.all_end_effector_p(3,start_M0:end_M0);
avg_z0=(M0_all_end_effector_p_z(1)-M0_all_end_effector_p_z(end))/size(M0_all_end_effector_p_z,2);
M0_all_EMG=M0.all_EMG(2:end-1,start_M0:end_M0);
avg_EMG_M0=sum(M0_all_EMG,2)/size(M0_all_EMG,2);

vx_M0=M0.all_end_effector_p(1,start_M0+1:end_M0+1)-M0.all_end_effector_p(1,start_M0-1:end_M0-1);
vz_M0=M0.all_end_effector_p(3,start_M0+1:end_M0+1)-M0.all_end_effector_p(3,start_M0-1:end_M0-1);
vx_M0=abs(vx_M0);
vz_M0=abs(vz_M0);

each_EMG_div_vz_vx_M0=sum(M0_all_EMG,1)./sum(vx_M0,1)./sum(vz_M0,1); % the power for each point
avg_each_EMG_div_vz_vx_M0=mean(each_EMG_div_vz_vx_M0);
std0_forv_div_vx=(1/length(each_EMG_div_vz_vx_M0)*sum((each_EMG_div_vz_vx_M0-mean(each_EMG_div_vz_vx_M0)).^2))^0.5;

each_EMG_div_vz_M0=sum(M0_all_EMG,1)./sum(vz_M0,1); % the power for each point
std0_forzv=(1/length(each_EMG_div_vz_M0)*sum((each_EMG_div_vz_M0-mean(each_EMG_div_vz_M0)).^2))^0.5;

%% z轴深度M1
start_M1=4145; end_M1=5846; 
M1_all_end_effector_p_z=M1.all_end_effector_p(3,start_M1:end_M1);
avg_z1=(M1_all_end_effector_p_z(1)-M1_all_end_effector_p_z(end))/size(M1_all_end_effector_p_z,2);
M1_all_EMG=M1.all_EMG(2:end-1,start_M1:end_M1);
avg_EMG_M1=sum(M1_all_EMG,2)/size(M1_all_EMG,2);

vz_M1=M1.all_end_effector_p(3,start_M1+1:end_M1+1)-M1.all_end_effector_p(3,start_M1-1:end_M1-1);
vx_M1=M1.all_end_effector_p(1,start_M1+1:end_M1+1)-M1.all_end_effector_p(1,start_M1-1:end_M1-1);
vx_M1=abs(vx_M1);
vz_M1=abs(vz_M1);
each_EMG_div_vxz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1)./sum(vx_M1,1); % the power for each point
avg_each_EMG_div_vz_vx_M1=mean(each_EMG_div_vxz_M1);
std1_forv_div_vx=(1/length(each_EMG_div_vxz_M1)*sum((each_EMG_div_vxz_M1-mean(each_EMG_div_vxz_M1)).^2))^0.5;

each_EMG_div_vz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1); % the power for each point
std1_forzv=(1/length(each_EMG_div_vz_M1)*sum((each_EMG_div_vz_M1-mean(each_EMG_div_vz_M1)).^2))^0.5;


%M2
start_state_M2_1=2750; start_state_M2_2=4759; 
M2_all_end_effector_p_z=M2.all_end_effector_p(3,start_state_M2_1:start_state_M2_2);
avg_z2=(M2_all_end_effector_p_z(1)-M2_all_end_effector_p_z(end))/size(M2_all_end_effector_p_z,2);

M2_all_EMG=M2.all_EMG(2:end-1,start_state_M2_1:start_state_M2_2);
avg_EMG_M2=sum(M2_all_EMG,2)/size(M2_all_EMG,2);

vx_M2=M2.all_end_effector_p(1,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(1,start_state_M2_1-1:start_state_M2_2-1);
vz_M2=M2.all_end_effector_p(3,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(3,start_state_M2_1-1:start_state_M2_2-1);
vx_M2=abs(vx_M2);
vz_M2=abs(vz_M2);

each_EMG_div_vxz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1)./sum(vx_M2,1); % the power for each point
avg_each_EMG_div_vz_vx_M2=mean(each_EMG_div_vxz_M2);
std2_forv_div_vx=(1/length(each_EMG_div_vxz_M2)*sum((each_EMG_div_vxz_M2-mean(each_EMG_div_vxz_M2)).^2))^0.5;

each_EMG_div_vz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1); % the power for each point
std2_forzv=(1/length(each_EMG_div_vz_M2)*sum((each_EMG_div_vz_M2-mean(each_EMG_div_vz_M2)).^2))^0.5;




% M3
start_state_M3_1=2923; start_state_M3_2=4129; 
M3_all_end_effector_p_z=M3.all_end_effector_p(3,start_state_M3_1:start_state_M3_2);
avg_z3=(M3_all_end_effector_p_z(1)-M3_all_end_effector_p_z(end))/size(M3_all_end_effector_p_z,2);

M3_all_EMG=M3.all_EMG(2:end-1,start_state_M3_1:start_state_M3_2);
avg_EMG_M3=sum(M3_all_EMG,2)/size(M3_all_EMG,2);

vx_M3=M3.all_end_effector_p(1,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(1,start_state_M3_1-1:start_state_M3_2-1);
vz_M3=M3.all_end_effector_p(3,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(3,start_state_M3_1-1:start_state_M3_2-1);
vx_M3=abs(vx_M3);
vz_M3=abs(vz_M3);

each_EMG_div_vxz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1)./sum(vx_M3,1); % the power for each point
avg_each_EMG_div_vz_vx_M3=mean(each_EMG_div_vxz_M3);
std3_forv_div_vx=(1/length(each_EMG_div_vxz_M3)*sum((each_EMG_div_vxz_M3-mean(each_EMG_div_vxz_M3)).^2))^0.5;

each_EMG_div_vz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1); % the power for each point
std3_forzv=(1/length(each_EMG_div_vz_M3)*sum((each_EMG_div_vz_M3-mean(each_EMG_div_vz_M3)).^2))^0.5;


sum_avg_EMG_M0=sum(avg_EMG_M0);
sum_avg_EMG_M1=sum(avg_EMG_M1);
sum_avg_EMG_M2=sum(avg_EMG_M2);
sum_avg_EMG_M3=sum(avg_EMG_M3);
avg_z0;
avg_z1;
avg_z2;
avg_z3;


power_div_z0=sum_avg_EMG_M0/avg_z0
power_div_z1=sum_avg_EMG_M1/avg_z1
power_div_z2=sum_avg_EMG_M2/avg_z2
power_div_z3=sum_avg_EMG_M3/avg_z3

std0_forv_div_vx
std1_forv_div_vx
std2_forv_div_vx
std3_forv_div_vx

std0_forzv
std1_forzv
std2_forzv
std3_forzv

avg_each_EMG_div_vz_vx_M0
avg_each_EMG_div_vz_vx_M1
avg_each_EMG_div_vz_vx_M2
avg_each_EMG_div_vz_vx_M3
end

if name == 6

M0=load('20220408zls_M0_fine.mat'); % 翘起来了。。。
M1=load('20220408zls_M1_good.mat');
M2=load('20220408zls_M2_good.mat');
M3=load('20220408zls_M3_good.mat');

M1_result=M1.result;
M2_result=M2.result;
M3_result=M3.result;

% EMG/(m/s)
%% z轴深度M0

start_M0=3724; end_M0=8148; 
M0_all_end_effector_p_z=M0.all_end_effector_p(3,start_M0:end_M0);
avg_z0=(M0_all_end_effector_p_z(1)-M0_all_end_effector_p_z(end))/size(M0_all_end_effector_p_z,2);
M0_all_EMG=M0.all_EMG(2:end-1,start_M0:end_M0);
avg_EMG_M0=sum(M0_all_EMG,2)/size(M0_all_EMG,2);

vx_M0=M0.all_end_effector_p(1,start_M0+1:end_M0+1)-M0.all_end_effector_p(1,start_M0-1:end_M0-1);
vz_M0=M0.all_end_effector_p(3,start_M0+1:end_M0+1)-M0.all_end_effector_p(3,start_M0-1:end_M0-1);
vx_M0=abs(vx_M0);
vz_M0=abs(vz_M0);

each_EMG_div_vz_vx_M0=sum(M0_all_EMG,1)./sum(vx_M0,1)./sum(vz_M0,1); % the power for each point
avg_each_EMG_div_vz_vx_M0=mean(each_EMG_div_vz_vx_M0);
std0_forv_div_vx=(1/length(each_EMG_div_vz_vx_M0)*sum((each_EMG_div_vz_vx_M0-mean(each_EMG_div_vz_vx_M0)).^2))^0.5;

each_EMG_div_vz_M0=sum(M0_all_EMG,1)./sum(vz_M0,1); % the power for each point
std0_forzv=(1/length(each_EMG_div_vz_M0)*sum((each_EMG_div_vz_M0-mean(each_EMG_div_vz_M0)).^2))^0.5;

%% z轴深度M1
start_M1=2711; end_M1=5041; 
M1_all_end_effector_p_z=M1.all_end_effector_p(3,start_M1:end_M1);
avg_z1=(M1_all_end_effector_p_z(1)-M1_all_end_effector_p_z(end))/size(M1_all_end_effector_p_z,2);
M1_all_EMG=M1.all_EMG(2:end-1,start_M1:end_M1);
avg_EMG_M1=sum(M1_all_EMG,2)/size(M1_all_EMG,2);

vz_M1=M1.all_end_effector_p(3,start_M1+1:end_M1+1)-M1.all_end_effector_p(3,start_M1-1:end_M1-1);
vx_M1=M1.all_end_effector_p(1,start_M1+1:end_M1+1)-M1.all_end_effector_p(1,start_M1-1:end_M1-1);
vx_M1=abs(vx_M1);
vz_M1=abs(vz_M1);
each_EMG_div_vxz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1)./sum(vx_M1,1); % the power for each point
avg_each_EMG_div_vz_vx_M1=mean(each_EMG_div_vxz_M1);
std1_forv_div_vx=(1/length(each_EMG_div_vxz_M1)*sum((each_EMG_div_vxz_M1-mean(each_EMG_div_vxz_M1)).^2))^0.5;

each_EMG_div_vz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1); % the power for each point
std1_forzv=(1/length(each_EMG_div_vz_M1)*sum((each_EMG_div_vz_M1-mean(each_EMG_div_vz_M1)).^2))^0.5;


%M2
start_state_M2_1=1995; start_state_M2_2=2235; 
M2_all_end_effector_p_z=M2.all_end_effector_p(3,start_state_M2_1:start_state_M2_2);
avg_z2=(M2_all_end_effector_p_z(1)-M2_all_end_effector_p_z(end))/size(M2_all_end_effector_p_z,2);

M2_all_EMG=M2.all_EMG(2:end-1,start_state_M2_1:start_state_M2_2);
avg_EMG_M2=sum(M2_all_EMG,2)/size(M2_all_EMG,2);

vx_M2=M2.all_end_effector_p(1,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(1,start_state_M2_1-1:start_state_M2_2-1);
vz_M2=M2.all_end_effector_p(3,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(3,start_state_M2_1-1:start_state_M2_2-1);
vx_M2=abs(vx_M2);
vz_M2=abs(vz_M2);

each_EMG_div_vxz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1)./sum(vx_M2,1); % the power for each point
avg_each_EMG_div_vz_vx_M2=mean(each_EMG_div_vxz_M2);
std2_forv_div_vx=(1/length(each_EMG_div_vxz_M2)*sum((each_EMG_div_vxz_M2-mean(each_EMG_div_vxz_M2)).^2))^0.5;

each_EMG_div_vz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1); % the power for each point
std2_forzv=(1/length(each_EMG_div_vz_M2)*sum((each_EMG_div_vz_M2-mean(each_EMG_div_vz_M2)).^2))^0.5;




% M3
start_state_M3_1=3468; start_state_M3_2=3799; 
M3_all_end_effector_p_z=M3.all_end_effector_p(3,start_state_M3_1:start_state_M3_2);
avg_z3=(M3_all_end_effector_p_z(1)-M3_all_end_effector_p_z(end))/size(M3_all_end_effector_p_z,2);

M3_all_EMG=M3.all_EMG(2:end-1,start_state_M3_1:start_state_M3_2);
avg_EMG_M3=sum(M3_all_EMG,2)/size(M3_all_EMG,2);

vx_M3=M3.all_end_effector_p(1,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(1,start_state_M3_1-1:start_state_M3_2-1);
vz_M3=M3.all_end_effector_p(3,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(3,start_state_M3_1-1:start_state_M3_2-1);
vx_M3=abs(vx_M3);
vz_M3=abs(vz_M3);

each_EMG_div_vxz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1)./sum(vx_M3,1); % the power for each point
avg_each_EMG_div_vz_vx_M3=mean(each_EMG_div_vxz_M3);
std3_forv_div_vx=(1/length(each_EMG_div_vxz_M3)*sum((each_EMG_div_vxz_M3-mean(each_EMG_div_vxz_M3)).^2))^0.5;

each_EMG_div_vz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1); % the power for each point
std3_forzv=(1/length(each_EMG_div_vz_M3)*sum((each_EMG_div_vz_M3-mean(each_EMG_div_vz_M3)).^2))^0.5;


sum_avg_EMG_M0=sum(avg_EMG_M0);
sum_avg_EMG_M1=sum(avg_EMG_M1);
sum_avg_EMG_M2=sum(avg_EMG_M2);
sum_avg_EMG_M3=sum(avg_EMG_M3);
avg_z0;
avg_z1;
avg_z2;
avg_z3;


power_div_z0=sum_avg_EMG_M0/avg_z0
power_div_z1=sum_avg_EMG_M1/avg_z1
power_div_z2=sum_avg_EMG_M2/avg_z2
power_div_z3=sum_avg_EMG_M3/avg_z3

std0_forv_div_vx
std1_forv_div_vx
std2_forv_div_vx
std3_forv_div_vx

std0_forzv
std1_forzv
std2_forzv
std3_forzv

avg_each_EMG_div_vz_vx_M0
avg_each_EMG_div_vz_vx_M1
avg_each_EMG_div_vz_vx_M2
avg_each_EMG_div_vz_vx_M3
end

if name == 7

M0=load('20220410zc_M0_good.mat'); % 翘起来了。。。
M1=load('20220410zc_M1_fine.mat');
M2=load('20220410zc_M2_fine.mat');
M3=load('20220410zc_M3_good.mat');

M1_result=M1.result;
M2_result=M2.result;
M3_result=M3.result;

% EMG/(m/s)
%% z轴深度M0

start_M0=4271; end_M0=6782; 
M0_all_end_effector_p_z=M0.all_end_effector_p(3,start_M0:end_M0);
avg_z0=(M0_all_end_effector_p_z(1)-M0_all_end_effector_p_z(end))/size(M0_all_end_effector_p_z,2);
M0_all_EMG=M0.all_EMG(2:end-1,start_M0:end_M0);
avg_EMG_M0=sum(M0_all_EMG,2)/size(M0_all_EMG,2);

vx_M0=M0.all_end_effector_p(1,start_M0+1:end_M0+1)-M0.all_end_effector_p(1,start_M0-1:end_M0-1);
vz_M0=M0.all_end_effector_p(3,start_M0+1:end_M0+1)-M0.all_end_effector_p(3,start_M0-1:end_M0-1);
vx_M0=abs(vx_M0);
vz_M0=abs(vz_M0);

each_EMG_div_vz_vx_M0=sum(M0_all_EMG,1)./sum(vx_M0,1)./sum(vz_M0,1); % the power for each point
avg_each_EMG_div_vz_vx_M0=mean(each_EMG_div_vz_vx_M0);
std0_forv_div_vx=(1/length(each_EMG_div_vz_vx_M0)*sum((each_EMG_div_vz_vx_M0-mean(each_EMG_div_vz_vx_M0)).^2))^0.5;

each_EMG_div_vz_M0=sum(M0_all_EMG,1)./sum(vz_M0,1); % the power for each point
std0_forzv=(1/length(each_EMG_div_vz_M0)*sum((each_EMG_div_vz_M0-mean(each_EMG_div_vz_M0)).^2))^0.5;

%% z轴深度M1
start_M1=4000; end_M1=7000; 
M1_all_end_effector_p_z=M1.all_end_effector_p(3,start_M1:end_M1);
avg_z1=(M1_all_end_effector_p_z(1)-M1_all_end_effector_p_z(end))/size(M1_all_end_effector_p_z,2);
M1_all_EMG=M1.all_EMG(2:end-1,start_M1:end_M1);
avg_EMG_M1=sum(M1_all_EMG,2)/size(M1_all_EMG,2);

vz_M1=M1.all_end_effector_p(3,start_M1+1:end_M1+1)-M1.all_end_effector_p(3,start_M1-1:end_M1-1);
vx_M1=M1.all_end_effector_p(1,start_M1+1:end_M1+1)-M1.all_end_effector_p(1,start_M1-1:end_M1-1);
vx_M1=abs(vx_M1);
vz_M1=abs(vz_M1);
each_EMG_div_vxz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1)./sum(vx_M1,1); % the power for each point
avg_each_EMG_div_vz_vx_M1=mean(each_EMG_div_vxz_M1);
std1_forv_div_vx=(1/length(each_EMG_div_vxz_M1)*sum((each_EMG_div_vxz_M1-mean(each_EMG_div_vxz_M1)).^2))^0.5;

each_EMG_div_vz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1); % the power for each point
std1_forzv=(1/length(each_EMG_div_vz_M1)*sum((each_EMG_div_vz_M1-mean(each_EMG_div_vz_M1)).^2))^0.5;


%M2
start_state_M2_1=1282; start_state_M2_2=2068; 
M2_all_end_effector_p_z=M2.all_end_effector_p(3,start_state_M2_1:start_state_M2_2);
avg_z2=(M2_all_end_effector_p_z(1)-M2_all_end_effector_p_z(end))/size(M2_all_end_effector_p_z,2);

M2_all_EMG=M2.all_EMG(2:end-1,start_state_M2_1:start_state_M2_2);
avg_EMG_M2=sum(M2_all_EMG,2)/size(M2_all_EMG,2);

vx_M2=M2.all_end_effector_p(1,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(1,start_state_M2_1-1:start_state_M2_2-1);
vz_M2=M2.all_end_effector_p(3,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(3,start_state_M2_1-1:start_state_M2_2-1);
vx_M2=abs(vx_M2);
vz_M2=abs(vz_M2);

each_EMG_div_vxz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1)./sum(vx_M2,1); % the power for each point
avg_each_EMG_div_vz_vx_M2=mean(each_EMG_div_vxz_M2);
std2_forv_div_vx=(1/length(each_EMG_div_vxz_M2)*sum((each_EMG_div_vxz_M2-mean(each_EMG_div_vxz_M2)).^2))^0.5;

each_EMG_div_vz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1); % the power for each point
std2_forzv=(1/length(each_EMG_div_vz_M2)*sum((each_EMG_div_vz_M2-mean(each_EMG_div_vz_M2)).^2))^0.5;




% M3
start_state_M3_1=3339; start_state_M3_2=5003; 
M3_all_end_effector_p_z=M3.all_end_effector_p(3,start_state_M3_1:start_state_M3_2);
avg_z3=(M3_all_end_effector_p_z(1)-M3_all_end_effector_p_z(end))/size(M3_all_end_effector_p_z,2);

M3_all_EMG=M3.all_EMG(2:end-1,start_state_M3_1:start_state_M3_2);
avg_EMG_M3=sum(M3_all_EMG,2)/size(M3_all_EMG,2);

vx_M3=M3.all_end_effector_p(1,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(1,start_state_M3_1-1:start_state_M3_2-1);
vz_M3=M3.all_end_effector_p(3,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(3,start_state_M3_1-1:start_state_M3_2-1);
vx_M3=abs(vx_M3);
vz_M3=abs(vz_M3);

each_EMG_div_vxz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1)./sum(vx_M3,1); % the power for each point
avg_each_EMG_div_vz_vx_M3=mean(each_EMG_div_vxz_M3);
std3_forv_div_vx=(1/length(each_EMG_div_vxz_M3)*sum((each_EMG_div_vxz_M3-mean(each_EMG_div_vxz_M3)).^2))^0.5;

each_EMG_div_vz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1); % the power for each point
std3_forzv=(1/length(each_EMG_div_vz_M3)*sum((each_EMG_div_vz_M3-mean(each_EMG_div_vz_M3)).^2))^0.5;


sum_avg_EMG_M0=sum(avg_EMG_M0);
sum_avg_EMG_M1=sum(avg_EMG_M1);
sum_avg_EMG_M2=sum(avg_EMG_M2);
sum_avg_EMG_M3=sum(avg_EMG_M3);
avg_z0;
avg_z1;
avg_z2;
avg_z3;


power_div_z0=sum_avg_EMG_M0/avg_z0
power_div_z1=sum_avg_EMG_M1/avg_z1
power_div_z2=sum_avg_EMG_M2/avg_z2
power_div_z3=sum_avg_EMG_M3/avg_z3

std0_forv_div_vx
std1_forv_div_vx
std2_forv_div_vx
std3_forv_div_vx

std0_forzv
std1_forzv
std2_forzv
std3_forzv

avg_each_EMG_div_vz_vx_M0
avg_each_EMG_div_vz_vx_M1
avg_each_EMG_div_vz_vx_M2
avg_each_EMG_div_vz_vx_M3
end

if name == 8

M0=load('20220420qqs_M0_good.mat'); % 翘起来了。。。
M1=load('20220420qqs_M1_good.mat');
M2=load('20220420qqs_M2_good.mat');
M3=load('20220420qqs_M3_bad.mat');

M1_result=M1.result;
M2_result=M2.result;
M3_result=M3.result;

% EMG/(m/s)
%% z轴深度M0

start_M0=4271; end_M0=6782; 
M0_all_end_effector_p_z=M0.all_end_effector_p(3,start_M0:end_M0);
avg_z0=(M0_all_end_effector_p_z(1)-M0_all_end_effector_p_z(end))/size(M0_all_end_effector_p_z,2);
M0_all_EMG=M0.all_EMG(2:end-1,start_M0:end_M0);
avg_EMG_M0=sum(M0_all_EMG,2)/size(M0_all_EMG,2);

vx_M0=M0.all_end_effector_p(1,start_M0+1:end_M0+1)-M0.all_end_effector_p(1,start_M0-1:end_M0-1);
vz_M0=M0.all_end_effector_p(3,start_M0+1:end_M0+1)-M0.all_end_effector_p(3,start_M0-1:end_M0-1);
vx_M0=abs(vx_M0);
vz_M0=abs(vz_M0);

each_EMG_div_vz_vx_M0=sum(M0_all_EMG,1)./sum(vx_M0,1)./sum(vz_M0,1); % the power for each point
avg_each_EMG_div_vz_vx_M0=mean(each_EMG_div_vz_vx_M0);
std0_forv_div_vx=(1/length(each_EMG_div_vz_vx_M0)*sum((each_EMG_div_vz_vx_M0-mean(each_EMG_div_vz_vx_M0)).^2))^0.5;

each_EMG_div_vz_M0=sum(M0_all_EMG,1)./sum(vz_M0,1); % the power for each point
std0_forzv=(1/length(each_EMG_div_vz_M0)*sum((each_EMG_div_vz_M0-mean(each_EMG_div_vz_M0)).^2))^0.5;

%% z轴深度M1
start_M1=3507; end_M1=7987; 
M1_all_end_effector_p_z=M1.all_end_effector_p(3,start_M1:end_M1);
avg_z1=(M1_all_end_effector_p_z(1)-M1_all_end_effector_p_z(end))/size(M1_all_end_effector_p_z,2);
M1_all_EMG=M1.all_EMG(2:end-1,start_M1:end_M1);
avg_EMG_M1=sum(M1_all_EMG,2)/size(M1_all_EMG,2);

vz_M1=M1.all_end_effector_p(3,start_M1+1:end_M1+1)-M1.all_end_effector_p(3,start_M1-1:end_M1-1);
vx_M1=M1.all_end_effector_p(1,start_M1+1:end_M1+1)-M1.all_end_effector_p(1,start_M1-1:end_M1-1);
vx_M1=abs(vx_M1);
vz_M1=abs(vz_M1);
each_EMG_div_vxz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1)./sum(vx_M1,1); % the power for each point
avg_each_EMG_div_vz_vx_M1=mean(each_EMG_div_vxz_M1);
std1_forv_div_vx=(1/length(each_EMG_div_vxz_M1)*sum((each_EMG_div_vxz_M1-mean(each_EMG_div_vxz_M1)).^2))^0.5;

each_EMG_div_vz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1); % the power for each point
std1_forzv=(1/length(each_EMG_div_vz_M1)*sum((each_EMG_div_vz_M1-mean(each_EMG_div_vz_M1)).^2))^0.5;


%M2
start_state_M2_1=3256; start_state_M2_2=3910; 
M2_all_end_effector_p_z=M2.all_end_effector_p(3,start_state_M2_1:start_state_M2_2);
avg_z2=(M2_all_end_effector_p_z(1)-M2_all_end_effector_p_z(end))/size(M2_all_end_effector_p_z,2);

M2_all_EMG=M2.all_EMG(2:end-1,start_state_M2_1:start_state_M2_2);
avg_EMG_M2=sum(M2_all_EMG,2)/size(M2_all_EMG,2);

vx_M2=M2.all_end_effector_p(1,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(1,start_state_M2_1-1:start_state_M2_2-1);
vz_M2=M2.all_end_effector_p(3,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(3,start_state_M2_1-1:start_state_M2_2-1);
vx_M2=abs(vx_M2);
vz_M2=abs(vz_M2);

each_EMG_div_vxz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1)./sum(vx_M2,1); % the power for each point
avg_each_EMG_div_vz_vx_M2=mean(each_EMG_div_vxz_M2);
std2_forv_div_vx=(1/length(each_EMG_div_vxz_M2)*sum((each_EMG_div_vxz_M2-mean(each_EMG_div_vxz_M2)).^2))^0.5;

each_EMG_div_vz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1); % the power for each point
std2_forzv=(1/length(each_EMG_div_vz_M2)*sum((each_EMG_div_vz_M2-mean(each_EMG_div_vz_M2)).^2))^0.5;




% M3
start_state_M3_1=1088; start_state_M3_2=1862; 
M3_all_end_effector_p_z=M3.all_end_effector_p(3,start_state_M3_1:start_state_M3_2);
avg_z3=(M3_all_end_effector_p_z(1)-M3_all_end_effector_p_z(end))/size(M3_all_end_effector_p_z,2);

M3_all_EMG=M3.all_EMG(2:end-1,start_state_M3_1:start_state_M3_2);
avg_EMG_M3=sum(M3_all_EMG,2)/size(M3_all_EMG,2);

vx_M3=M3.all_end_effector_p(1,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(1,start_state_M3_1-1:start_state_M3_2-1);
vz_M3=M3.all_end_effector_p(3,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(3,start_state_M3_1-1:start_state_M3_2-1);
vx_M3=abs(vx_M3);
vz_M3=abs(vz_M3);

each_EMG_div_vxz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1)./sum(vx_M3,1); % the power for each point
avg_each_EMG_div_vz_vx_M3=mean(each_EMG_div_vxz_M3);
std3_forv_div_vx=(1/length(each_EMG_div_vxz_M3)*sum((each_EMG_div_vxz_M3-mean(each_EMG_div_vxz_M3)).^2))^0.5;

each_EMG_div_vz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1); % the power for each point
std3_forzv=(1/length(each_EMG_div_vz_M3)*sum((each_EMG_div_vz_M3-mean(each_EMG_div_vz_M3)).^2))^0.5;


sum_avg_EMG_M0=sum(avg_EMG_M0);
sum_avg_EMG_M1=sum(avg_EMG_M1);
sum_avg_EMG_M2=sum(avg_EMG_M2);
sum_avg_EMG_M3=sum(avg_EMG_M3);
avg_z0;
avg_z1;
avg_z2;
avg_z3;


power_div_z0=sum_avg_EMG_M0/avg_z0
power_div_z1=sum_avg_EMG_M1/avg_z1
power_div_z2=sum_avg_EMG_M2/avg_z2
power_div_z3=sum_avg_EMG_M3/avg_z3

std0_forv_div_vx
std1_forv_div_vx
std2_forv_div_vx
std3_forv_div_vx

std0_forzv
std1_forzv
std2_forzv
std3_forzv

avg_each_EMG_div_vz_vx_M0
avg_each_EMG_div_vz_vx_M1
avg_each_EMG_div_vz_vx_M2
avg_each_EMG_div_vz_vx_M3
end

if name == 9

M0=load('20220421zyn_M0_good.mat'); % 翘起来了。。。
M1=load('20220421zyn_M1_good.mat');
M2=load('20220421zyn_M2_fine.mat');
M3=load('20220421zyn_M3_fine.mat');

M1_result=M1.result;
M2_result=M2.result;
M3_result=M3.result;

% EMG/(m/s)
%% z轴深度M0

start_M0=3816; end_M0=7044; 
M0_all_end_effector_p_z=M0.all_end_effector_p(3,start_M0:end_M0);
avg_z0=(M0_all_end_effector_p_z(1)-M0_all_end_effector_p_z(end))/size(M0_all_end_effector_p_z,2);
M0_all_EMG=M0.all_EMG(2:end-1,start_M0:end_M0);
avg_EMG_M0=sum(M0_all_EMG,2)/size(M0_all_EMG,2);

vx_M0=M0.all_end_effector_p(1,start_M0+1:end_M0+1)-M0.all_end_effector_p(1,start_M0-1:end_M0-1);
vz_M0=M0.all_end_effector_p(3,start_M0+1:end_M0+1)-M0.all_end_effector_p(3,start_M0-1:end_M0-1);
vx_M0=abs(vx_M0);
vz_M0=abs(vz_M0);

each_EMG_div_vz_vx_M0=sum(M0_all_EMG,1)./sum(vx_M0,1)./sum(vz_M0,1); % the power for each point
avg_each_EMG_div_vz_vx_M0=mean(each_EMG_div_vz_vx_M0);
std0_forv_div_vx=(1/length(each_EMG_div_vz_vx_M0)*sum((each_EMG_div_vz_vx_M0-mean(each_EMG_div_vz_vx_M0)).^2))^0.5;

each_EMG_div_vz_M0=sum(M0_all_EMG,1)./sum(vz_M0,1); % the power for each point
std0_forzv=(1/length(each_EMG_div_vz_M0)*sum((each_EMG_div_vz_M0-mean(each_EMG_div_vz_M0)).^2))^0.5;

%% z轴深度M1
start_M1=3056; end_M1=8095; 
M1_all_end_effector_p_z=M1.all_end_effector_p(3,start_M1:end_M1);
avg_z1=(M1_all_end_effector_p_z(1)-M1_all_end_effector_p_z(end))/size(M1_all_end_effector_p_z,2);
M1_all_EMG=M1.all_EMG(2:end-1,start_M1:end_M1);
avg_EMG_M1=sum(M1_all_EMG,2)/size(M1_all_EMG,2);

vz_M1=M1.all_end_effector_p(3,start_M1+1:end_M1+1)-M1.all_end_effector_p(3,start_M1-1:end_M1-1);
vx_M1=M1.all_end_effector_p(1,start_M1+1:end_M1+1)-M1.all_end_effector_p(1,start_M1-1:end_M1-1);
vx_M1=abs(vx_M1);
vz_M1=abs(vz_M1);
each_EMG_div_vxz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1)./sum(vx_M1,1); % the power for each point
avg_each_EMG_div_vz_vx_M1=mean(each_EMG_div_vxz_M1);
std1_forv_div_vx=(1/length(each_EMG_div_vxz_M1)*sum((each_EMG_div_vxz_M1-mean(each_EMG_div_vxz_M1)).^2))^0.5;

each_EMG_div_vz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1); % the power for each point
std1_forzv=(1/length(each_EMG_div_vz_M1)*sum((each_EMG_div_vz_M1-mean(each_EMG_div_vz_M1)).^2))^0.5;


%M2
start_state_M2_1=3986; start_state_M2_2=4507; 
M2_all_end_effector_p_z=M2.all_end_effector_p(3,start_state_M2_1:start_state_M2_2);
avg_z2=(M2_all_end_effector_p_z(1)-M2_all_end_effector_p_z(end))/size(M2_all_end_effector_p_z,2);

M2_all_EMG=M2.all_EMG(2:end-1,start_state_M2_1:start_state_M2_2);
avg_EMG_M2=sum(M2_all_EMG,2)/size(M2_all_EMG,2);

vx_M2=M2.all_end_effector_p(1,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(1,start_state_M2_1-1:start_state_M2_2-1);
vz_M2=M2.all_end_effector_p(3,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(3,start_state_M2_1-1:start_state_M2_2-1);
vx_M2=abs(vx_M2);
vz_M2=abs(vz_M2);

each_EMG_div_vxz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1)./sum(vx_M2,1); % the power for each point
avg_each_EMG_div_vz_vx_M2=mean(each_EMG_div_vxz_M2);
std2_forv_div_vx=(1/length(each_EMG_div_vxz_M2)*sum((each_EMG_div_vxz_M2-mean(each_EMG_div_vxz_M2)).^2))^0.5;

each_EMG_div_vz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1); % the power for each point
std2_forzv=(1/length(each_EMG_div_vz_M2)*sum((each_EMG_div_vz_M2-mean(each_EMG_div_vz_M2)).^2))^0.5;




% M3
start_state_M3_1=3280; start_state_M3_2=3775; 
M3_all_end_effector_p_z=M3.all_end_effector_p(3,start_state_M3_1:start_state_M3_2);
avg_z3=(M3_all_end_effector_p_z(1)-M3_all_end_effector_p_z(end))/size(M3_all_end_effector_p_z,2);

M3_all_EMG=M3.all_EMG(2:end-1,start_state_M3_1:start_state_M3_2);
avg_EMG_M3=sum(M3_all_EMG,2)/size(M3_all_EMG,2);

vx_M3=M3.all_end_effector_p(1,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(1,start_state_M3_1-1:start_state_M3_2-1);
vz_M3=M3.all_end_effector_p(3,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(3,start_state_M3_1-1:start_state_M3_2-1);
vx_M3=abs(vx_M3);
vz_M3=abs(vz_M3);

each_EMG_div_vxz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1)./sum(vx_M3,1); % the power for each point
avg_each_EMG_div_vz_vx_M3=mean(each_EMG_div_vxz_M3);
std3_forv_div_vx=(1/length(each_EMG_div_vxz_M3)*sum((each_EMG_div_vxz_M3-mean(each_EMG_div_vxz_M3)).^2))^0.5;

each_EMG_div_vz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1); % the power for each point
std3_forzv=(1/length(each_EMG_div_vz_M3)*sum((each_EMG_div_vz_M3-mean(each_EMG_div_vz_M3)).^2))^0.5;


sum_avg_EMG_M0=sum(avg_EMG_M0);
sum_avg_EMG_M1=sum(avg_EMG_M1);
sum_avg_EMG_M2=sum(avg_EMG_M2);
sum_avg_EMG_M3=sum(avg_EMG_M3);
avg_z0;
avg_z1;
avg_z2;
avg_z3;


power_div_z0=sum_avg_EMG_M0/avg_z0
power_div_z1=sum_avg_EMG_M1/avg_z1
power_div_z2=sum_avg_EMG_M2/avg_z2
power_div_z3=sum_avg_EMG_M3/avg_z3

std0_forv_div_vx
std1_forv_div_vx
std2_forv_div_vx
std3_forv_div_vx

std0_forzv
std1_forzv
std2_forzv
std3_forzv

avg_each_EMG_div_vz_vx_M0
avg_each_EMG_div_vz_vx_M1
avg_each_EMG_div_vz_vx_M2
avg_each_EMG_div_vz_vx_M3
end


if name == 10

M0=load('20220421bry_M0_good.mat'); % 翘起来了。。。
M1=load('20220421bry_M1_good.mat');
M2=load('20220421bry_M2_fine.mat');
M3=load('20220421bry_M3_fine.mat');

M1_result=M1.result;
M2_result=M2.result;
M3_result=M3.result;

% EMG/(m/s)
%% z轴深度M0

start_M0=6000; end_M0=8000; 
M0_all_end_effector_p_z=M0.all_end_effector_p(3,start_M0:end_M0);
avg_z0=(M0_all_end_effector_p_z(1)-M0_all_end_effector_p_z(end))/size(M0_all_end_effector_p_z,2);
M0_all_EMG=M0.all_EMG(2:end-1,start_M0:end_M0);
avg_EMG_M0=sum(M0_all_EMG,2)/size(M0_all_EMG,2);

vx_M0=M0.all_end_effector_p(1,start_M0+1:end_M0+1)-M0.all_end_effector_p(1,start_M0-1:end_M0-1);
vz_M0=M0.all_end_effector_p(3,start_M0+1:end_M0+1)-M0.all_end_effector_p(3,start_M0-1:end_M0-1);
vx_M0=abs(vx_M0);
vz_M0=abs(vz_M0);

each_EMG_div_vz_vx_M0=sum(M0_all_EMG,1)./sum(vx_M0,1)./sum(vz_M0,1); % the power for each point
avg_each_EMG_div_vz_vx_M0=mean(each_EMG_div_vz_vx_M0);
std0_forv_div_vx=(1/length(each_EMG_div_vz_vx_M0)*sum((each_EMG_div_vz_vx_M0-mean(each_EMG_div_vz_vx_M0)).^2))^0.5;

each_EMG_div_vz_M0=sum(M0_all_EMG,1)./sum(vz_M0,1); % the power for each point
std0_forzv=(1/length(each_EMG_div_vz_M0)*sum((each_EMG_div_vz_M0-mean(each_EMG_div_vz_M0)).^2))^0.5;

%% z轴深度M1
start_M1=6245; end_M1=8275; 
M1_all_end_effector_p_z=M1.all_end_effector_p(3,start_M1:end_M1);
avg_z1=(M1_all_end_effector_p_z(1)-M1_all_end_effector_p_z(end))/size(M1_all_end_effector_p_z,2);
M1_all_EMG=M1.all_EMG(2:end-1,start_M1:end_M1);
avg_EMG_M1=sum(M1_all_EMG,2)/size(M1_all_EMG,2);

vz_M1=M1.all_end_effector_p(3,start_M1+1:end_M1+1)-M1.all_end_effector_p(3,start_M1-1:end_M1-1);
vx_M1=M1.all_end_effector_p(1,start_M1+1:end_M1+1)-M1.all_end_effector_p(1,start_M1-1:end_M1-1);
vx_M1=abs(vx_M1);
vz_M1=abs(vz_M1);
each_EMG_div_vxz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1)./sum(vx_M1,1); % the power for each point
avg_each_EMG_div_vz_vx_M1=mean(each_EMG_div_vxz_M1);
std1_forv_div_vx=(1/length(each_EMG_div_vxz_M1)*sum((each_EMG_div_vxz_M1-mean(each_EMG_div_vxz_M1)).^2))^0.5;

each_EMG_div_vz_M1=sum(M1_all_EMG,1)./sum(vz_M1,1); % the power for each point
std1_forzv=(1/length(each_EMG_div_vz_M1)*sum((each_EMG_div_vz_M1-mean(each_EMG_div_vz_M1)).^2))^0.5;


%M2
start_state_M2_1=1256; start_state_M2_2=1552; 
M2_all_end_effector_p_z=M2.all_end_effector_p(3,start_state_M2_1:start_state_M2_2);
avg_z2=(M2_all_end_effector_p_z(1)-M2_all_end_effector_p_z(end))/size(M2_all_end_effector_p_z,2);

M2_all_EMG=M2.all_EMG(2:end-1,start_state_M2_1:start_state_M2_2);
avg_EMG_M2=sum(M2_all_EMG,2)/size(M2_all_EMG,2);

vx_M2=M2.all_end_effector_p(1,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(1,start_state_M2_1-1:start_state_M2_2-1);
vz_M2=M2.all_end_effector_p(3,start_state_M2_1+1:start_state_M2_2+1)-M2.all_end_effector_p(3,start_state_M2_1-1:start_state_M2_2-1);
vx_M2=abs(vx_M2);
vz_M2=abs(vz_M2);

each_EMG_div_vxz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1)./sum(vx_M2,1); % the power for each point
avg_each_EMG_div_vz_vx_M2=mean(each_EMG_div_vxz_M2);
std2_forv_div_vx=(1/length(each_EMG_div_vxz_M2)*sum((each_EMG_div_vxz_M2-mean(each_EMG_div_vxz_M2)).^2))^0.5;

each_EMG_div_vz_M2=sum(M2_all_EMG,1)./sum(vz_M2,1); % the power for each point
std2_forzv=(1/length(each_EMG_div_vz_M2)*sum((each_EMG_div_vz_M2-mean(each_EMG_div_vz_M2)).^2))^0.5;




% M3
start_state_M3_1=2375; start_state_M3_2=2591; 
M3_all_end_effector_p_z=M3.all_end_effector_p(3,start_state_M3_1:start_state_M3_2);
avg_z3=(M3_all_end_effector_p_z(1)-M3_all_end_effector_p_z(end))/size(M3_all_end_effector_p_z,2);

M3_all_EMG=M3.all_EMG(2:end-1,start_state_M3_1:start_state_M3_2);
avg_EMG_M3=sum(M3_all_EMG,2)/size(M3_all_EMG,2);

vx_M3=M3.all_end_effector_p(1,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(1,start_state_M3_1-1:start_state_M3_2-1);
vz_M3=M3.all_end_effector_p(3,start_state_M3_1+1:start_state_M3_2+1)-M3.all_end_effector_p(3,start_state_M3_1-1:start_state_M3_2-1);
vx_M3=abs(vx_M3);
vz_M3=abs(vz_M3);

each_EMG_div_vxz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1)./sum(vx_M3,1); % the power for each point
avg_each_EMG_div_vz_vx_M3=mean(each_EMG_div_vxz_M3);
std3_forv_div_vx=(1/length(each_EMG_div_vxz_M3)*sum((each_EMG_div_vxz_M3-mean(each_EMG_div_vxz_M3)).^2))^0.5;

each_EMG_div_vz_M3=sum(M3_all_EMG,1)./sum(vz_M3,1); % the power for each point
std3_forzv=(1/length(each_EMG_div_vz_M3)*sum((each_EMG_div_vz_M3-mean(each_EMG_div_vz_M3)).^2))^0.5;


sum_avg_EMG_M0=sum(avg_EMG_M0);
sum_avg_EMG_M1=sum(avg_EMG_M1);
sum_avg_EMG_M2=sum(avg_EMG_M2);
sum_avg_EMG_M3=sum(avg_EMG_M3);
avg_z0;
avg_z1;
avg_z2;
avg_z3;


power_div_z0=sum_avg_EMG_M0/avg_z0
power_div_z1=sum_avg_EMG_M1/avg_z1
power_div_z2=sum_avg_EMG_M2/avg_z2
power_div_z3=sum_avg_EMG_M3/avg_z3

std0_forv_div_vx
std1_forv_div_vx
std2_forv_div_vx
std3_forv_div_vx

std0_forzv
std1_forzv
std2_forzv
std3_forzv

avg_each_EMG_div_vz_vx_M0
avg_each_EMG_div_vz_vx_M1
avg_each_EMG_div_vz_vx_M2
avg_each_EMG_div_vz_vx_M3
end





result1=[avg_each_EMG_div_vz_vx_M0; avg_each_EMG_div_vz_vx_M1; avg_each_EMG_div_vz_vx_M2; avg_each_EMG_div_vz_vx_M3];
result2=[std0_forv_div_vx; std1_forv_div_vx; std2_forv_div_vx; std3_forv_div_vx;];
result3=[std0_forzv; std1_forzv; std2_forzv; std3_forzv];
result4=[power_div_z0; power_div_z1; power_div_z2; power_div_z3];
results=[result1 result2 result3 result4];

allresults=[allresults; results];
end