close all;

init_1=1/6;init_2=1/6;init_3=1/6;init_4=1/6;init_5=1/6;init_6=1/6;
lambda=1;
result=[]; result_y=[];result_y_upstd=[];result_y_downstd=[];
%%
prior_intent=each_intent;
X_test = each_test_file;
%%
pt20=[X_test(1,1:end) prior_intent];
pt10=[X_test(1,1:end) prior_intent];
pt30=[X_test(1,1:end) prior_intent];
pt40=[X_test(1,1:end) prior_intent];
pt50=[X_test(1,1:end) prior_intent];
pt60=[X_test(1,1:end) prior_intent];
result_yci=[];
result=[];
for t = 1:length(X_test)-1
    this_den=0;
    g_relate_pre=[];
    max_yci=[];
    all_rou=[];
    for g = 1:6
        expr_g=['pt' num2str(g) '0'];
        value_g=eval(expr_g);
        expr_g_init=['init_' num2str(g)];
        init_g=eval(expr_g_init);       
        thisg_pre=[];
        each_yci=[];
        rou0=1;
        for GPR = 1:13
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
    all_init=[];  
    for g = 1:6
        expr_g00=['init_' num2str(g)];
        init_g=eval(expr_g00);      
        rou00=all_rou(g);
        this_mem=rou00*init_g^lambda;
        next_rou_value=this_mem/this_den;
        all_init=[all_init next_rou_value];
        expr_rou=['init_' num2str(g) ' = next_rou_value;' ];
        eval(expr_rou);
    end   
    
    %比大小
     which_max=find(all_init==max(all_init));
     if length(which_max)>1
         which_max=which_max(1);
     end
     this_pre=g_relate_pre(which_max,:);
     this_yci=max_yci(which_max,:);
     for g = 1:6
            expr_newpt=['pt' num2str(g) '0 = [this_pre g];' ];
            eval(expr_newpt);
     end
     result=[result; this_pre;];
     result_yci=[result_yci; this_yci;];
end



figure(1);
plot(result(:,1)); hold on; plot(each_test_file_Y(2:end,1)); hold on;
plot(result_yci(:,1),'k--')
plot(result_yci(:,2),'k--')
legend('predict z','True value z','Prediction interval limits');

figure(2);
plot(result(:,2)); hold on; plot(each_test_file_Y(2:end,2)); hold on;
plot(result_yci(:,3),'k--')
plot(result_yci(:,4),'k--')
legend('predict y','True value y','Prediction interval limits');

figure(3);
plot(result(:,3)); hold on; plot(each_test_file_Y(2:end,3)); hold on;
plot(result_yci(:,5),'k--')
plot(result_yci(:,6),'k--')
legend('predict x','True value x','Prediction interval limits');


