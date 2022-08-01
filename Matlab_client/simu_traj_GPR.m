close all;clear;clc;

version = 2;
%% 这个版本只用主轴，相当于预测两次
if version == 2
    ullld=[0.6 0.6 0.6 0.6 0.6; -0.3 -0.3 0 0.3 0.3; 0.25 0.525 0.525 0.525 0.25;];
    way_points=ullld;
    BIGTIME=2; Ts= 0.01;
    T_tot=(size(way_points,2)-1)*BIGTIME
    
    tvec = 0:Ts:T_tot;
    tpts = 0:T_tot/(size(way_points,2)-1):T_tot;
    
    [points,points_dot,points_dotdot,pp] = cubicpolytraj(way_points,tpts,tvec,...
        'VelocityBoundaryCondition', zeros(3,5));
    figure;plot(points(2,:),points(3,:))
    
    train_X_y=points(2,1+200:end-1-200).';
    train_X_z=points(3,1:end-1).';
    train_Y_y=points(2,2+200:end-200).';
    train_Y_z=points(3,2:end).';
    train_X_z = [train_X_z 1*ones(size(train_X_z,1),1)];
    train_X_y = [train_X_y 1*ones(size(train_X_y,1),1)];
    
    gprMdl_y1 = fitrgp(train_X_y,train_Y_y);
    gprMdl_z1 = fitrgp(train_X_z,train_Y_z);
    [ypred_f, ~, yci_f] = predict(gprMdl_y1, train_X_y );
    
    figure(41)
    plot(train_X_y(:,1) ,gprMdl_y1.Y,'r.'); hold on;
    plot(train_X_y ,ypred_f, 'y.'); hold on;
    plot(train_X_y, yci_f(:,1),'k:'); hold on;
    plot(train_X_y, yci_f(:,2),'k:');
    
    last_start=-0.3;
    tic
    [result,result_yci] = myGRP_onlyx_limitfuture(last_start, 1, gprMdl_y1,400);
    toc
    figure(42)
    plot(gprMdl_y1.Y,'r.'); hold on;
    % plot(ypred_f, 'y.'); hold on;
    plot(result, 'b.'); hold on;
    % plot(yci_f(:,1),'k:'); hold on;
    % plot(yci_f(:,2),'k:');hold on;
    plot(result_yci(:,1),'k:'); hold on;
    plot(result_yci(:,2),'k:');hold on;
    legend('train data','predicted value with all','GPR plus bayes')
    
end

%% 这个版本是xyz都用上的
if version == 1
ullld=[0.6 0.6 0.6 0.6 0.6; -0.3 -0.3 0 0.3 0.3; 0.25 0.525 0.525 0.525 0.25;];
way_points=ullld;
BIGTIME=2; Ts= 0.01;
T_tot=(size(way_points,2)-1)*BIGTIME

tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(way_points,2)-1):T_tot;

[points,points_dot,points_dotdot,pp] = cubicpolytraj(way_points,tpts,tvec,...
                'VelocityBoundaryCondition', zeros(3,5));   
figure;plot(points(2,:),points(3,:))

train_X_xyz=points(:,1:end-1).';
train_Y_x=points(1,2:end).';
train_Y_y=points(2,2:end).';
train_Y_z=points(3,2:end).';
train_X_xyz = [train_X_xyz 1*ones(size(train_X_xyz,1),1)];

% time=1:length(train_X_xyz);
% train_X_xyz = [train_X_xyz time.'];
gprMdl_x1 = fitrgp(train_X_xyz,train_Y_x);
gprMdl_y1 = fitrgp(train_X_xyz,train_Y_y);
gprMdl_z1 = fitrgp(train_X_xyz,train_Y_z);

[ypred_f, ~, yci_f] = predict(gprMdl_z1, train_X_xyz );

% figure(41)
% plot(train_X_xyz(:,end) ,gprMdl_z1.Y,'r.'); hold on;
% plot(train_X_xyz(:,end) ,ypred_f, 'y.'); hold on;
% plot(train_X_xyz(:,end), yci_f(:,1),'k:'); hold on;
% plot(train_X_xyz(:,end), yci_f(:,2),'k:');
% max_range=max(train_X_xyz1(:,1)); min_range=min(train_X_xyz1(:,1));
% plot([min_range, max_range] , [min_range, max_range] ,'g')
tic
[result,result_yci] = GRP_xyz_onepoint(train_X_xyz(1,1:3), 1, gprMdl_x1,gprMdl_y1,gprMdl_z1,800);
toc
figure(41)
plot(gprMdl_y1.Y,'r.'); hold on;
plot(result(:,2), 'y.'); hold on;
plot(result_yci(:,3),'k:'); hold on;
plot(result_yci(:,4),'k:');
legend('train data','predicted value')

figure(42)
plot(gprMdl_z1.Y,'r.'); hold on;
plot(result(:,3), 'y.'); hold on;
plot(result_yci(:,5),'k:'); hold on;
plot(result_yci(:,6),'k:');
legend('train data','predicted value')

end


function [result,result_yci] = GRP_xyz_onepoint(X_test, prior_intent, gprMdl_1,gprMdl_2,gprMdl_3,predict_length)
 

    init_1=1/2;init_2=1/2;
    lambda=0.8;num_intent=1;
    result=[]; result_y=[];result_y_upstd=[];result_y_downstd=[];
    nmb_p2=[];
    %%
    pt20=[X_test(1,1:end) prior_intent];
    pt10=[X_test(1,1:end) prior_intent];
    result_yci=[];
    result=[X_test(1,:)];
    for t = 1:predict_length
        this_den=0;
        g_relate_pre=[];
        max_yci=[];
        all_rou=[];
        for g = 1:num_intent
            expr_g=['pt' num2str(g) '0'];
            value_g=eval(expr_g);
            expr_g_init=['init_' num2str(g)];
            init_g=eval(expr_g_init);       
            thisg_pre=[];
            each_yci=[];
            rou0=1;
            for GPR = 1:3
                expr_GPR=['gprMdl_' num2str(GPR)];
                gprMdl=eval(expr_GPR);
                [ypred_single, standard, yci] = predict(gprMdl,value_g); 
                each_yci=[each_yci yci];
                expr_pre_y=['pre_y_' num2str(GPR) ' = ypred_single;' ];
                eval(expr_pre_y);
                thisg_pre=[thisg_pre ypred_single];
                rou=normpdf(ypred_single,ypred_single,standard);
                rou0=rou0*rou;
            end
            %%
            g_relate_pre=[g_relate_pre; thisg_pre;];
            all_rou=[all_rou rou0];
            max_yci=[max_yci; each_yci;];
            this_den=this_den+rou0*init_g^lambda;
        end
        all_rou;
        all_init=[];  
        for g = 1:num_intent
            expr_g00=['init_' num2str(g)];
            init_g=eval(expr_g00);      
            rou00=all_rou(g);
            this_mem=rou00*init_g^lambda;
            next_rou_value=this_mem/this_den;
            all_init=[all_init next_rou_value];
            expr_rou=['init_' num2str(g) ' = next_rou_value;' ];
            eval(expr_rou);
        end   
all_init;
        %比大小
         which_max=find(all_init==max(all_init));
         if length(which_max)>1
             which_max=which_max(1);
         end
         this_pre=g_relate_pre(which_max,:);

         this_yci=max_yci(which_max,:);
         for g = 1:num_intent
                expr_newpt=['pt' num2str(g) '0 = [this_pre g];' ];
                eval(expr_newpt);
         end
pt10;
pt20;
nmb_p2=[nmb_p2; pt20;];
         result=[result; this_pre;];
         result_yci=[result_yci; this_yci;];
    end


end




function [result,result_yci] = myGRP_onlyx_limitfuture(X_test, prior_intent, gprMdl_1,how_long)
 % only use the motion on main axis to predict the future motion on
 % mainaxis
    init_1=1/2;init_2=1/2;
    lambda=0.8;num_intent=2;
    result=[]; result_y=[];result_y_upstd=[];result_y_downstd=[];
    nmb_p2=[];
    %%
    pt20=[X_test(1,1:end) prior_intent];
    pt10=[X_test(1,1:end) prior_intent];
    result_yci=[];
    result=[X_test(1,:)];
    for t = 1:how_long
        this_den=0;
        g_relate_pre=[];
        max_yci=[];
        all_rou=[];
        for g = 1:num_intent
            expr_g=['pt' num2str(g) '0'];
            value_g=eval(expr_g);
            expr_g_init=['init_' num2str(g)];
            init_g=eval(expr_g_init);       
            thisg_pre=[];
            each_yci=[];
            rou0=1;
            for GPR = 1:1
                expr_GPR=['gprMdl_' num2str(GPR)];
                gprMdl=eval(expr_GPR);

                [ypred_single, standard, yci] = predict(gprMdl,value_g); 
                
                
                
                
                each_yci=[each_yci yci];
                expr_pre_y=['pre_y_' num2str(GPR) ' = ypred_single;' ];
                eval(expr_pre_y);
                thisg_pre=[thisg_pre ypred_single];
                rou=normpdf(ypred_single,ypred_single,standard);
                rou0=rou0*rou;
            end

            %%
            g_relate_pre=[g_relate_pre; thisg_pre;];
            all_rou=[all_rou rou0];
            max_yci=[max_yci; each_yci;];
            this_den=this_den+rou0*init_g^lambda;
        end
        all_rou;
        all_init=[];  
        for g = 1:num_intent
            expr_g00=['init_' num2str(g)];
            init_g=eval(expr_g00);      
            rou00=all_rou(g);
            this_mem=rou00*init_g^lambda;
            next_rou_value=this_mem/this_den;
            all_init=[all_init next_rou_value];
            expr_rou=['init_' num2str(g) ' = next_rou_value;' ];
            eval(expr_rou);
        end   
all_init;
        %比大小
         which_max=find(all_init==max(all_init));
         if length(which_max)>1
             which_max=which_max(1);
         end
         this_pre=g_relate_pre(which_max,:);

         this_yci=max_yci(which_max,:);
         for g = 1:num_intent
                expr_newpt=['pt' num2str(g) '0 = [this_pre g];' ];
                eval(expr_newpt);
         end
pt10;
pt20;
nmb_p2=[nmb_p2; pt20;];
         result=[result; this_pre;];
         result_yci=[result_yci; this_yci;];
    end


end