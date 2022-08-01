function result = myGRP_onlyx(X_test, prior_intent, gprMdl_1)
 % only use the motion on main axis to predict the future motion on
 % mainaxis, 注意不要用eval写法



    init_1=1/2;init_2=1/2;
    lambda=0.8;num_intent=2;
    result=[]; result_y=[];result_y_upstd=[];result_y_downstd=[];
    nmb_p2=[];
    %%
    pt20=[X_test(1,1:end) prior_intent];
    pt10=[X_test(1,1:end) prior_intent];
    result_yci=[];
    result=[X_test(1,:)];
    for t = 1:size(X_test,1)-1


        this_den=0;
        g_relate_pre=[];
        max_yci=[];
        all_rou=[];

%         for g = 1:num_intent
%             expr_g=['pt' num2str(g) '0'];
%             value_g=eval(expr_g);
%             expr_g_init=['init_' num2str(g)];
%             init_g=eval(expr_g_init);       
%             thisg_pre=[];
%             each_yci=[];
%             rou0=1;
%                 [ypred_single, standard, yci] = predict(gprMdl_1,value_g); 
%                 each_yci=[each_yci yci];
%                 pre_y_1 = ypred_single;
%                 thisg_pre=[thisg_pre ypred_single];
%                 rou=normpdf(ypred_single,ypred_single,standard);
%                 rou0=rou0*rou;
%             %%
%             g_relate_pre=[g_relate_pre; thisg_pre;];
%             all_rou=[all_rou rou0];
%             max_yci=[max_yci; each_yci;];
%             this_den=this_den+rou0*init_g^lambda;
%         end
     



value_g=pt10;
init_g=init_1;
thisg_pre=[];
each_yci=[];
rou0=1;
[ypred_single, standard, yci] = predict(gprMdl_1,value_g);
each_yci=[each_yci yci];
pre_y_1 = ypred_single;
thisg_pre=[thisg_pre ypred_single];
rou=normpdf(ypred_single,ypred_single,standard);
rou0=rou0*rou;
g_relate_pre=[g_relate_pre; thisg_pre;];
all_rou=[all_rou rou0];
max_yci=[max_yci; each_yci;];
this_den=this_den+rou0*init_g^lambda;

value_g=pt20;
init_g=init_2;
thisg_pre=[];
each_yci=[];
rou0=1;
[ypred_single, standard, yci] = predict(gprMdl_1,value_g);
each_yci=[each_yci yci];
pre_y_1 = ypred_single;
thisg_pre=[thisg_pre ypred_single];
rou=normpdf(ypred_single,ypred_single,standard);
rou0=rou0*rou;
g_relate_pre=[g_relate_pre; thisg_pre;];
all_rou=[all_rou rou0];
max_yci=[max_yci; each_yci;];
this_den=this_den+rou0*init_g^lambda;

        

        all_rou;
        all_init=[];  
        
%         for g = 1:num_intent
%             expr_g00=['init_' num2str(g)];
%             init_g=eval(expr_g00);      
%             rou00=all_rou(g);
%             this_mem=rou00*init_g^lambda;
%             next_rou_value=this_mem/this_den;
%             all_init=[all_init next_rou_value];
%             expr_rou=['init_' num2str(g) ' = next_rou_value;' ];
%             eval(expr_rou);
%         end   


        rou00=all_rou(1);
        this_mem=rou00*init_1^lambda;
        next_rou_value=this_mem/this_den;
        all_init=[all_init next_rou_value];        
        init_1 = next_rou_value;
        
        rou00=all_rou(2);
        this_mem=rou00*init_2^lambda;
        next_rou_value=this_mem/this_den;
        all_init=[all_init next_rou_value];        
        init_2 = next_rou_value;        
        
        
        
        %比大小
         which_max=find(all_init==max(all_init));
         if length(which_max)>1
             which_max=which_max(1);
         end
         this_pre=g_relate_pre(which_max,:);

         this_yci=max_yci(which_max,:);
         

        pt10= [this_pre 1];
        pt20= [this_pre 2];

         nmb_p2=[nmb_p2; pt20;];
         result=[result; this_pre;];
         result_yci=[result_yci; this_yci;];

         
         
    end


end