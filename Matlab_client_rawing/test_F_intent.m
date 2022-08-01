close all;clear;clc;
load('20220306M2_jiuchayidia.mat')
% have_EMG=data_all(8,3354:end);
result=[]; count_others=1;
for look = 1:length(come_in)
    if come_in(look) == 4
        result=[result; [4 4 ...
            4 FUCKact(look)/2 come_in(look)];];        
    else
        
        result=[result; [all_end_effector_p(1,count_others)*3 new_f(1,count_others)/100 ...
            output(count_others) FUCKact(look)/2 come_in(look)];];
        count_others=count_others+1;
    end
end
sample=result(2000:3000,:);
figure(8);plot(sample)
legend('real position','contatc F','intent','act','hold it')

int_3=find(sample(:,2) > 0);
int_2=find(sample(:,2) < -0);
init=zeros(size(sample,1),1);
init(int_3) = 3;init(int_2) = -3;

correct=find(init/2 == sample(:,end-1));
length(correct)
figure(32)
plot(init/2); hold on;
plot( sample(:,end-1))

% figure;plot(data_all(8,3354:end)); hold on;

% figure;
% plot(all_end_effector_p(1,:)); hold on;
% plot(new_f(1,:)/100); hold on;
% plot(output)
% legend('real x motion','real contact force','


