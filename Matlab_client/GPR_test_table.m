close all;clear;clc;
% 目前，还有相机合并，避障，GPR，意图，IMU整合 需要确认

%1前 2后 3上 4下 5左 6右 0停
%% 用csv
% file1=readmatrix('20220331_lpms1.csv');
% file2=readmatrix('20220331_lpms2.csv');
% file3=readmatrix('20220331_lpms3.csv');
% figure(1)
% plot(file2(:,13));
% figure(2)
% plot(file2(:,14));
% figure(3)
% plot(file2(:,15));


% file1=file1(2123:9000,13:15);
% file2=file2(2123:9000,13:15);
% file3=file3(2123:9000,13:15);
% 
% file1=file1-file1(1,:);
% file2=file2-file2(1,:);
% file3=file3-file3(1,:);
% all_y21=[];all_a21=[];all_b21=[];
% all_y23=[];all_a23=[];all_b23=[];
% all_y31=[];all_a31=[];all_b31=[];
% init_wrist=[0; 45; 0;];
% result=[init_wrist];
% 
% for fuckyou=1:size(file1,1)
%     this1=[zeros(1,12) file1(fuckyou,1) file1(fuckyou,2) file1(fuckyou,3)];
%     this2=[zeros(1,12) file2(fuckyou,1) file2(fuckyou,2) file2(fuckyou,3)];
%     this3=[zeros(1,12) file3(fuckyou,1) file3(fuckyou,2) file3(fuckyou,3)];
%     R1 = rotate_matrix(this1);
%     R2 = rotate_matrix(this2);
%     R3 = rotate_matrix(this3);
%     R31 = R1.'*R3;
%     R23 = R3.'*R2;
%     R21 = R1.'*R2;
%             [b21,a21,y21] = inverse_angle(R21);
%             [b23,a23,y23] = inverse_angle(R23);
%             [b31,a31,y31] = inverse_angle(R31);
%             all_a21=[all_a21 a21];
%             all_b21=[all_b21 b21];
%             all_y21=[all_y21 y21];    
%             all_a23=[all_a23 a23];
%             all_b23=[all_b23 b23];
%             all_y23=[all_y23 y23];   
%             all_a31=[all_a31 a31];
%             all_b31=[all_b31 b31];
%             all_y31=[all_y31 y31];              
%             
%     result=[result R31*[0; 20; 0;]+R23*[0; 25; 0;]];
% end


% figure(12)
% plot(all_a21)
% figure(1)
% plot(result(1,:)); hold on;

figure;
plot(all_y23); hold on;
plot(all_b23); hold on;
plot(all_a23)
legend('y23','b23','a23')

figure;
plot(all_y21); hold on;
plot(all_b21); hold on;
plot(all_a21)
legend('y21','b21','a21')


SVM_angles=0;
% 用mat
% load('standardize_IMU.mat')
%% SVM with input angles
if SVM_angles == 1
for sfile =1:2
    if sfile==1
        load('pull2_IMU0406.mat');
    else
        load('pull1_IMU0406.mat');
    end
my_start=find(all_a21==100)+1;
init_wrist_x=0; init_wrist_y=25; init_wrist_z=0;
init_wrist_3=[init_wrist_x;init_wrist_y;init_wrist_z;];
result_wrist1=[];
% all_y21=[];all_a21=[];all_b21=[];
% all_y23=[];all_a23=[];all_b23=[];
% all_y31=[];all_a31=[];all_b31=[];
% my_angles=[]
% for k = 1:size(data_all_IMU,2)
%     this_frame_IMU=data_all_IMU(:,k)-data_all_IMU(:,1);
%     
%     
%     angles=this_frame_IMU([14:16 39:41 64:66]);
%     
%     
%     too_big=find(angles > 320);
%     too_small=find(angles < -320);
%     angles(too_big)=angles(too_big)-360;
%     angles(too_small)=angles(too_small)+360;
%     my_angles=[my_angles angles];
%     
%     
%     this1=[zeros(1,12) angles(1:3).'];
%     this2=[zeros(1,12) angles(4:6).'];
%     this3=[zeros(1,12) angles(7:9).'];
% %     this1=this_frame_IMU(1+1:25+1,:).';
% %     this2=this_frame_IMU(26+1:50+1,:).';
% %     this3=this_frame_IMU(51+1:75+1,:).';
%     R1 = rotate_matrix(this1);
%     R2 = rotate_matrix(this2);
%     R3 = rotate_matrix(this3);
%     R31 = R1.'*R3;
%     R23 = R3.'*R2;
%     R21 = R1.'*R2;
%             [b21,a21,y21] = inverse_angle(R21);
%             [b23,a23,y23] = inverse_angle(R23);
%             [b31,a31,y31] = inverse_angle(R31);
%             all_a21=[all_a21 a21];
%             all_b21=[all_b21 b21];
%             all_y21=[all_y21 y21];    
%             all_a23=[all_a23 a23];
%             all_b23=[all_b23 b23];
%             all_y23=[all_y23 y23];   
%             all_a31=[all_a31 a31];
%             all_b31=[all_b31 b31];
%             all_y31=[all_y31 y31];      
%     
%     temp_result= R31*[0; 20; 0;]+R23*[0; 25; 0;];
%     result_wrist=[result_wrist temp_result];
% end
% figure(1);
% plot(result_wrist(1,:)); hold on;
% figure(2);
% plot(result_wrist(2,:)); hold on;
% figure(3);
% plot(result_wrist(3,:)); hold on;

if sfile == 1
intent1=find(all_contact_force_after(1,:) <= -15);
intent2=find(all_contact_force_after(1,:) >= 15);
intent4=find(all_contact_force_after(2,:) <= -15);
intent3=find(all_contact_force_after(2,:) >= 15);
intent5=find(all_contact_force_after(3,:) <= -15);
intent6=find(all_contact_force_after(3,:) >= 15);
all_a21=(all_a21-all_a21(my_start));
all_b21=(all_b21-all_b21(my_start));
all_y21=(all_y21-all_y21(my_start));
all_a21=all_a21(my_start:end);
all_b21=all_b21(my_start:end);
all_y21=all_y21(my_start:end);
all_a31=(all_a31-all_a31(my_start));
all_b31=(all_b31-all_b31(my_start));
all_y31=(all_y31-all_y31(my_start));
all_a31=all_a31(my_start:end);
all_b31=all_b31(my_start:end);
all_y31=all_y31(my_start:end);
all_a23=(all_a23-all_a23(my_start));
all_b23=(all_b23-all_b23(my_start));
all_y23=(all_y23-all_y23(my_start));
all_a23=all_a23(my_start:end);
all_b23=all_b23(my_start:end);
all_y23=all_y23(my_start:end);

left=length(all_contact_force_after);
ori_group=zeros(1,left);

ori_group(intent1)=1;ori_group(intent2)=2;ori_group(intent3)=3;
ori_group(intent4)=4;ori_group(intent5)=5;ori_group(intent6)=6;


training_X=[all_a23; all_b23; all_y23; all_a31; all_b31; all_y31; all_a23; all_b23; all_y23;].';
group=ori_group;

X=training_X;
Y=group;
classOrder = unique(Y);
% t = templateSVM('Standardize',true);
t = templateSVM('KernelFunction','gaussian');
PMdl = fitcecoc(X,Y,'Holdout',0.010,'Learners',t,'ClassNames',classOrder);
Mdl = PMdl.Trained{1};           % Extract trained, compact classifier

sample=X;
test_answer=group;

outclass = predict(Mdl,sample);
delta=outclass-test_answer.';
correct=find(delta==0);
answer=length(correct)/size(sample,1)
%draw sth
figure(1) % 
plot(test_answer); hold on;
plot(outclass)

end

%% 测试
if sfile == 2
my_start=find(all_a21==100)+1;
all_a21=(all_a21-all_a21(my_start));
all_b21=(all_b21-all_b21(my_start));
all_y21=(all_y21-all_y21(my_start));
all_a21=all_a21(my_start:end);
all_b21=all_b21(my_start:end);
all_y21=all_y21(my_start:end);
all_a31=(all_a31-all_a31(my_start));
all_b31=(all_b31-all_b31(my_start));
all_y31=(all_y31-all_y31(my_start));
all_a31=all_a31(my_start:end);
all_b31=all_b31(my_start:end);
all_y31=all_y31(my_start:end);
all_a23=(all_a23-all_a23(my_start));
all_b23=(all_b23-all_b23(my_start));
all_y23=(all_y23-all_y23(my_start));
all_a23=all_a23(my_start:end);
all_b23=all_b23(my_start:end);
all_y23=all_y23(my_start:end);

intent1=find(all_contact_force_after(1,:) <= -15);
intent2=find(all_contact_force_after(1,:) >= 15);
intent4=find(all_contact_force_after(2,:) <= -15);
intent3=find(all_contact_force_after(2,:) >= 15);
intent5=find(all_contact_force_after(3,:) <= -15);
intent6=find(all_contact_force_after(3,:) >= 15);
left=length(all_contact_force_after);
ori_group=zeros(1,left);

ori_group(intent1)=1;ori_group(intent2)=2;ori_group(intent3)=3;
ori_group(intent4)=4;ori_group(intent5)=5;ori_group(intent6)=6;

sample=[all_a23; all_b23; all_y23; all_a31; all_b31; all_y31; all_a23; all_b23; all_y23;].';
test_answer=ori_group;

outclass = predict(Mdl,sample);
delta=outclass-test_answer.';
correct=find(delta==0);
answer=length(correct)/size(sample,1)
figure(2)
plot(test_answer); hold on;
plot(outclass)


end
end
end

%% SVM with input angles
for sfile =1:1
    if sfile==1
        load('pull2_IMU0406.mat');
    else
        load('pull1_IMU0406.mat');
    end
my_start=find(all_a21==100)+1;
init_wrist_x=0; init_wrist_y=25; init_wrist_z=0;
init_wrist_3=[init_wrist_x;init_wrist_y;init_wrist_z;];
result_wrist=[];

% figure(41);
% plot(all_a21(my_start:end)); hold on;
% figure(42);
% plot(all_b21(my_start:end)); hold on;
% figure(43);
% plot(all_y21(my_start:end)); hold on;

% all_y21=[];all_a21=[];all_b21=[];
% all_y23=[];all_a23=[];all_b23=[];
% all_y31=[];all_a31=[];all_b31=[];
my_angles=[]
for k = 1:size(data_all_IMU,2)
%     this_frame_IMU=data_all_IMU(:,k)-data_all_IMU(:,1); 
    this_frame_IMU=data_all_IMU(:,k);
    angles=this_frame_IMU([14:16 39:41 64:66]);
    my_angles=[my_angles angles];
    

%     this1=this_frame_IMU(1+1:25+1,:).';
%     this2=this_frame_IMU(26+1:50+1,:).';
%     this3=this_frame_IMU(51+1:75+1,:).';
%     R1 = rotate_matrix(this1);
%     R2 = rotate_matrix(this2);
%     R3 = rotate_matrix(this3);
%     R31 = R1.'*R3;
%     R23 = R3.'*R2;
%     R21 = R1.'*R2;
%     [b21,a21,y21] = inverse_angle(R21);
%     all_a21=[all_a21 a21];
%     all_b21=[all_b21 b21];
%     all_y21=[all_y21 y21];
            
    
    this1=[zeros(1,12) angles(1:3).'];
    this2=[zeros(1,12) angles(4:6).'];
    this3=[zeros(1,12) angles(7:9).'];

    R1 = rotate_matrix(this1);
    R2 = rotate_matrix(this2);
    R3 = rotate_matrix(this3);
    R31 = R1.'*R3;
    R23 = R3.'*R2;
    R21 = R1.'*R2;
    temp_result= R31*[0; 20; 0;]+R23*[0; 25; 0;];
    result_wrist=[result_wrist temp_result];
end

% figure(41);
% plot(all_a21); hold on;
% figure(42);
% plot(all_b21); hold on;
% figure(43);
% plot(all_y21); hold on;



figure(1);
plot(result_wrist(1,:)); hold on;
figure(2);
plot(result_wrist(2,:)); hold on;
figure(3);
plot(result_wrist(3,:)); hold on;

if sfile == 1
intent1=find(all_contact_force_after(1,:) <= -15);
intent2=find(all_contact_force_after(1,:) >= 15);
intent4=find(all_contact_force_after(2,:) <= -15);
intent3=find(all_contact_force_after(2,:) >= 15);
intent5=find(all_contact_force_after(3,:) <= -15);
intent6=find(all_contact_force_after(3,:) >= 15);
all_a21=(all_a21-all_a21(my_start));
all_b21=(all_b21-all_b21(my_start));
all_y21=(all_y21-all_y21(my_start));
all_a21=all_a21(my_start:end);
all_b21=all_b21(my_start:end);
all_y21=all_y21(my_start:end);
all_a31=(all_a31-all_a31(my_start));
all_b31=(all_b31-all_b31(my_start));
all_y31=(all_y31-all_y31(my_start));
all_a31=all_a31(my_start:end);
all_b31=all_b31(my_start:end);
all_y31=all_y31(my_start:end);
all_a23=(all_a23-all_a23(my_start));
all_b23=(all_b23-all_b23(my_start));
all_y23=(all_y23-all_y23(my_start));
all_a23=all_a23(my_start:end);
all_b23=all_b23(my_start:end);
all_y23=all_y23(my_start:end);

left=length(all_contact_force_after);
ori_group=zeros(1,left);

ori_group(intent1)=1;ori_group(intent2)=2;ori_group(intent3)=3;
ori_group(intent4)=4;ori_group(intent5)=5;ori_group(intent6)=6;
result_wrist1=[];
for k = 1:size(all_b23,2)

    this23=[zeros(1,12) all_y23(k) all_b23(k) all_a23(k)];
    this31=[zeros(1,12) all_y31(k) all_b31(k) all_a31(k)];

    R23 = rotate_matrix(this23);
    R31 = rotate_matrix(this31);

    temp_result= R31*[0; 20; 0;]+R23*[0; 25; 0;];
    result_wrist1=[result_wrist1 temp_result];
end
training_X=result_wrist1.';
% training_X=[all_a23; all_b23; all_y23; all_a31; all_b31; all_y31; all_a21; all_b21; all_y21;].';
group=ori_group;

X=training_X;
Y=group;
classOrder = unique(Y);
% t = templateSVM('Standardize',true);
t = templateSVM('KernelFunction','gaussian');
PMdl = fitcecoc(X,Y,'Holdout',0.010,'Learners',t,'ClassNames',classOrder);
Mdl = PMdl.Trained{1};           % Extract trained, compact classifier

sample=X;
test_answer=group;

outclass = predict(Mdl,sample);
delta=outclass-test_answer.';
correct=find(delta==0);
answer=length(correct)/size(sample,1)
%draw sth
figure(11) % 
plot(test_answer); hold on;
plot(outclass)
figure(1);
plot(result_wrist1(1,:)); hold on;
figure(2);
plot(result_wrist1(2,:)); hold on;
figure(3);
plot(result_wrist1(3,:)); hold on;
end

%% 测试
if sfile == 2
my_start=find(all_a21==100)+1;
all_a21=(all_a21-all_a21(my_start));
all_b21=(all_b21-all_b21(my_start));
all_y21=(all_y21-all_y21(my_start));
all_a21=all_a21(my_start:end);
all_b21=all_b21(my_start:end);
all_y21=all_y21(my_start:end);
all_a31=(all_a31-all_a31(my_start));
all_b31=(all_b31-all_b31(my_start));
all_y31=(all_y31-all_y31(my_start));
all_a31=all_a31(my_start:end);
all_b31=all_b31(my_start:end);
all_y31=all_y31(my_start:end);
all_a23=(all_a23-all_a23(my_start));
all_b23=(all_b23-all_b23(my_start));
all_y23=(all_y23-all_y23(my_start));
all_a23=all_a23(my_start:end);
all_b23=all_b23(my_start:end);
all_y23=all_y23(my_start:end);

intent1=find(all_contact_force_after(1,:) <= -15);
intent2=find(all_contact_force_after(1,:) >= 15);
intent4=find(all_contact_force_after(2,:) <= -15);
intent3=find(all_contact_force_after(2,:) >= 15);
intent5=find(all_contact_force_after(3,:) <= -15);
intent6=find(all_contact_force_after(3,:) >= 15);
left=length(all_contact_force_after);
ori_group=zeros(1,left);

ori_group(intent1)=1;ori_group(intent2)=2;ori_group(intent3)=3;
ori_group(intent4)=4;ori_group(intent5)=5;ori_group(intent6)=6;
result_wrist2=[];
for k = 1:size(all_b23,2)

    this23=[zeros(1,12) all_y23(k) all_b23(k) all_a23(k)];
    this31=[zeros(1,12) all_y31(k) all_b31(k) all_a31(k)];

    R23 = rotate_matrix(this23);
    R31 = rotate_matrix(this31);

    temp_result= R31*[0; 20; 0;]+R23*[0; 25; 0;];
    result_wrist2=[result_wrist2 temp_result];
end
sample=result_wrist2.';

% sample=[all_a23; all_b23; all_y23; all_a31; all_b31; all_y31; all_a21; all_b21; all_y21;].';
test_answer=ori_group;

outclass = predict(Mdl,sample);
delta=outclass-test_answer.';
correct=find(delta==0);
answer=length(correct)/size(sample,1)
figure(12)
plot(test_answer); hold on;
plot(outclass)


end
end



