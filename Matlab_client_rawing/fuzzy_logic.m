close all;clear;clc;
warning('off')

upp=load('upward.mat');
up=upp.sychronize;
mixx=load('mix333.mat');
mix=mixx.sychronize;
fowrr=load('forward.mat');
forw=fowrr.sychronize;
backk=load('backward.mat');
back=backk.sychronize;
test_mix=mix(4:end,:);
% test_mix_1=test_mix(:,1);
qda=[up(4:end,:) forw(4:end,:) back(4:end,:)]; 
answer=[1*ones(1,size(up,2)) 2*ones(1,size(forw,2)) 3*ones(1,size(back,2))];


[y1, ps]=mapminmax(qda,0,1);
y2=mapminmax('apply',test_mix,ps);

%% up
x1 = y2(2,:).';
x2 = y2(3,:).';
y = mix(3,:).';
X = [ones(size(x1)) x1 x2];
[b,bint] = regress(y,X)
scatter3(x1,x2,y,'filled')
hold on
x1fit = min(x1):0.01:max(x1);
x2fit = min(x2):0.01:max(x2);
[X1FIT,X2FIT] = meshgrid(x1fit,x2fit);
YFIT = b(1)+ b(2)*X1FIT+b(3)*X2FIT;
mesh(X1FIT,X2FIT,YFIT)
xlabel('x1')
ylabel('x2')
zlabel('Y')
view(140,30)


% % Set up fittype and options. 
% ft = fittype('a./(1+exp(-b*x))', 'independent', 'x', 'dependent', 'y'); 
% opts = fitoptions('Method', 'NonlinearLeastSquares'); 
% opts.Display = 'Off'; 
% opts.StartPoint = [0.957 0.481 0.957 0.481 0.957 0.481 0.953]; 
% F_x=mix(1,:);
% Fit model to data. 
% [fitresult, gof] = fit(y2.', , ft, opts); 

%%
% % after_all_EMG_forw=forw(4:end,:).';
% after_all_EMG_forw=y1(:,3002:6002).';
% all_F_forw=forw(1,:).';
% [coef_1_forw,fitinfo_forw] = lasso(after_all_EMG_forw,all_F_forw,'Weights',ones(size(after_all_EMG_forw,1),1),'Alpha',0.75,'Lambda',0.005);
% pre_y=after_all_EMG_forw*coef_1_forw+fitinfo_forw.Intercept;
% 
% % after_all_EMG_back=up(4:end,:).';
% after_all_EMG_back=y1(:,6003:9003).';
% all_F_back=back(1,:).';
% [coef_1_back,fitinfo_back] = lasso(after_all_EMG_back,all_F_back,'Weights',ones(size(after_all_EMG_back,1),1),'Alpha',0.75,'Lambda',0.005);
% pre_y=after_all_EMG_back*coef_1_back+fitinfo_back.Intercept;
% 
% % after_all_EMG_up=up(4:end,:).';
% after_all_EMG_up=y1(:,1:3001).';
% all_F_up=up(3,:).';
% [coef_1_up,fitinfo_up] = lasso(after_all_EMG_up,all_F_up,'Weights',ones(size(after_all_EMG_up,1),1),'Alpha',0.75,'Lambda',0.005);
% pre_y=after_all_EMG_up*coef_1_up+fitinfo_up.Intercept;
% confuse=y2.';


% output=[];predict_F=[];
% for z = 1:size(y2,2)
%     this_frame=y2(:,z).';
% 
%    if this_frame(3)>0.3 && this_frame(2)>0.05
%        
%         output=[output 1];
%         pre_y=this_frame*coef_1_up+fitinfo_up.Intercept;
%      elseif this_frame(5)>0.4
%         output=[output 2];
%         pre_y=this_frame*coef_1_forw+fitinfo_forw.Intercept;       
%         
%         
%     elseif this_frame(7)>0.2 || this_frame(6)>0.3
%         output=[output 3];        
%         pre_y=this_frame*coef_1_back+fitinfo_back.Intercept;
%     else
%         output=[output 0]; 
%         pre_y=0;
%     end
%     predict_F=[predict_F pre_y];
% end
% figure(90)
% plot(y2(2,:),mix(1,:));
% 
% output=[];predict_F=[];
% for z = 1:size(y2,2)
%     this_frame=y2(:,z).';
%     out3=sigmoid(this_frame(2),1);
%     output=[output out3];
%     out6=sigmoid(this_frame(6),10);
% 
%     out5=sigmoid(this_frame(5),10);
%     
%     pre_y=this_frame*coef_1_up+fitinfo_up.Intercept;
%     predict_F=[predict_F pre_y];
% 
% end
% 
% 
% % figure;plot(output)
% plot(predict_F)



function output = sigmoid(x,siz)
xx=x*siz;
output =1./(1+exp(-xx));
end





% %% Create the robot object
% ip='172.31.1.147'; % The IP of the controller
% arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
% arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
% Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
% iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
% addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main-raw')  
% 
% %% Start a connection with the server
% flag=iiwa.net_establishConnection();
% if flag==0
%   return;
% end
% pause(1);
% disp('Moving first joint of the robot using a sinusoidal function')
%     
% 
% %% Go to initial position
% relVel=0.15; % the relative velocity
% % theta_points=[-0.401300000000000;0.953900000000000;0;-1.04900000000000;0;1.13740000000000;0].';
% % theta_points=[-0.463647609000806,0.820126730911849,0,-1.17997019621945,0,1.14149572645849,0];
% my_t=zeros(1,7);
% [All_theta] = inverse_with_gesture(0.65,0,0.2,180,0).';
% jj=All_theta(:,2)*pi/180;
% jj(end)=0;
% theta_points=jj.';
% theta_points2=num2cell(theta_points)
% iiwa.movePTPJointSpace(theta_points2, relVel); % move to initial configuration
% all_jpos=[];
% %%
% iiwa.realTime_startVelControlJoints();all_output=[];all_F_contact=[];new_torque=[];
% for q = 1 : 100
%     q
%     if q~=1
%         my_torque=cell2mat(my_t).';
%         new_torque=[new_torque; my_torque;];
%         F_contact=1*pinv(J_dx_dq.')*my_torque;
%         all_F_contact=[all_F_contact F_contact];       %%!!! 
%     end
%         this_p=iiwa.getJointsPos();
%     this_p=cell2mat(this_p);
%             [Jac,A_mat_products] = Jacobian(this_p,1);
%         J_dx_dq = Jac(1:3,:);
%             this_p=iiwa.getJointsPos();
%     this_p=cell2mat(this_p);
%     
%     all_jpos=[all_jpos; this_p;];  
%     
%     
%     
%     feedback_joint_velocity_after=zeros(1,7);
%     this_zukang=num2cell(feedback_joint_velocity_after);
%     my_t=iiwa.sendJointsVelocitiesExTorques(this_zukang);
%  
% end
% iiwa.realTime_stopVelControlJoints();
% iiwa.net_turnOffServer()


