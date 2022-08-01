function [result]= updateGPR_M3_onlyx(total_act,start_point, gprMdl_x1_1, gprMdl_x2_1)




if total_act(1)-total_act(2) == 3
    each_intent =1; last_intent = 2;
    new_gprMdl_1=gprMdl_x1_1;    
elseif total_act(1)-total_act(2) == -3
    each_intent =2; last_intent = 1;
    new_gprMdl_1=gprMdl_x2_1;

end
          
        result = myGRP_onlyx_limitfuture(start_point, each_intent, new_gprMdl_1);
        

end