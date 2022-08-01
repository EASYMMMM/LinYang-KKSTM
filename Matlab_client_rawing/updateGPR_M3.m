function [result]= updateGPR_M3(total_act,start_point, gprMdl_x1_1, gprMdl_y1_1, gprMdl_z1_1, gprMdl_x2_1, gprMdl_y2_1, gprMdl_z2_1)




if total_act(1)-total_act(2) == 3
    each_intent =1; last_intent = 2;
    new_gprMdl_1=gprMdl_x1_1;
    new_gprMdl_2=gprMdl_y1_1;
    new_gprMdl_3=gprMdl_z1_1;
    
 
    
elseif total_act(1)-total_act(2) == -3
    each_intent =2; last_intent = 1;
    new_gprMdl_1=gprMdl_x2_1;
    new_gprMdl_2=gprMdl_y2_1;
    new_gprMdl_3=gprMdl_z2_1;
    
    
end


%         if each_intent == 1 
%              temp_combine_X2=[each_test_file_add; train_X_xyz1;];
%              temp_combine_Y2=[each_test_file_Y_add; train_Y_xyz1;];
% 
%               gprMdl_x2 = updateGPRMdl(gprMdl_x2, temp_combine_X2,  temp_combine_Y2(:,1));
%               gprMdl_y2 = updateGPRMdl(gprMdl_y2, temp_combine_X2,  temp_combine_Y2(:,2));
%               gprMdl_z2 = updateGPRMdl(gprMdl_z2, temp_combine_X2,  temp_combine_Y2(:,3));
% 
%         elseif each_intent == 2
%              temp_combine_X1=[each_test_file_add; train_X_xyz2;];
%              temp_combine_Y1=[each_test_file_Y_add; train_Y_xyz2;];          
% 
%  
%               gprMdl_x1 = updateGPRMdl(gprMdl_x1, temp_combine_X1,  temp_combine_Y1(:,1));
%               gprMdl_y1 = updateGPRMdl(gprMdl_y1, temp_combine_X1,  temp_combine_Y1(:,2));
%               gprMdl_z1 = updateGPRMdl(gprMdl_z1, temp_combine_X1,  temp_combine_Y1(:,3));
% 
%         end

tic               
        result = GRP_xyz_onepoint(start_point, each_intent, new_gprMdl_1,new_gprMdl_2,new_gprMdl_3);
toc
end