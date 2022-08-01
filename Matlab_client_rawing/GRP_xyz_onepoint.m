function result = GRP_xyz_onepoint(X_test, prior_intent, gprMdl_1,gprMdl_2,gprMdl_3)
 
% init_1=0;init_2=0;init_3=0;init_4=0;init_5=0;init_6=0;
% if prior_intent == 1
%     init_1 = 1;
% elseif prior_intent == 2
%     init_2 = 1;
% elseif prior_intent == 3
%     init_3 = 1;
% elseif prior_intent == 4
%     init_4 = 1;
% elseif prior_intent == 5
%     init_5 = 1;
% elseif prior_intent == 6
%     init_6 = 1;
% end

    init_1=1/2;init_2=1/2;
    lambda=0.8;num_intent=2;
    result=[]; result_y=[];result_y_upstd=[];result_y_downstd=[];
    nmb_p2=[];
    %%
    pt20=[X_test(1,1:end) prior_intent];
    pt10=[X_test(1,1:end) prior_intent];
    result_yci=[];
    result=[X_test(1,:)];
    for t = 1:100
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