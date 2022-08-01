close all;clear;clc;
% use GPR to predict

load('20220119fial.mat');
roundfrom1=[all_end_effector_p(:,4000:end)]; 
roundfrom2=[all_end_effector_p(:,4000:end)]; 

R=0.005;
kalmanz = dsp.KalmanFilter('ProcessNoiseCovariance', 0.0001,...
    'MeasurementNoiseCovariance',R,...
    'InitialStateEstimate',5,...
    'InitialErrorCovarianceEstimate',1,...
    'ControlInputPort',false); %Create Kalman filter
% 
% coef_1=load('LASSO_coef.mat').coef_1;
% coef_2=load('LASSO_coef.mat').coef_2;
% fitinfo2=load('LASSO_coef.mat').fitinfo2;
% fitinfo1=load('LASSO_coef.mat').fitinfo1;
% Mdl=load('LASSO_coef.mat').Mdl;

% round1= load ('20220109yl5.mat').sychronize;  %
% round2= load ('20220109yl6.mat').sychronize;  %
% round3= load ('20220109yl4.mat').sychronize;
% round4= load ('20220109yl7.mat').sychronize;
% round5= load ('20220109yl7.mat').sychronize;
% round6= load ('20220109yl6.mat').sychronize;
% round7= load ('20220109yl7.mat').sychronize;

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
    

    
  %%  
  
    sele_st=1; sele_end=7;


            temp_X=[];temp_Y=[];
            for shit = 1:length(v)-1
                this = v(shit);
                next = v(shit+1);
                if this > 0
                    intent_t=1;
                else
                    intent_t=2;
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

%             figure(89)
%             plot(temp_X(1,:)); hold on; plot(temp_Y(1,:)); 
            
     if file <= sep
            
        this_round_train_X=temp_X;
        this_round_train_Y=temp_Y;
        all_round_thisd_train_X=[all_round_thisd_train_X; this_round_train_X.'];
        all_round_thisd_train_Y=[all_round_thisd_train_Y; this_round_train_Y.'];        

    else


        this_round_test_X=temp_X;
        this_round_test_Y=temp_Y;
        all_round_thisd_test_X=[all_round_thisd_test_X; this_round_test_X.'];
        all_round_thisd_test_Y=[all_round_thisd_test_Y; this_round_test_Y.'];        


    end

    all_round_train_X=[all_round_train_X; all_round_thisd_train_X];
    all_round_train_Y=[all_round_train_Y; all_round_thisd_train_Y];
    
    all_round_test_X=[all_round_test_X; all_round_thisd_test_X];
    all_round_test_Y=[all_round_test_Y; all_round_thisd_test_Y];    


end