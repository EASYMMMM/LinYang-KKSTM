function result = updateGPR(total_act, this_eef_pos,way_points)



file_GPR2 = 'gprMdl20220127_2.mat';
gprMdl_x1=load(file_GPR2).gprMdl_x1;
gprMdl_y1=load(file_GPR2).gprMdl_y1;
gprMdl_z1=load(file_GPR2).gprMdl_z1;
gprMdl_x2=load(file_GPR2).gprMdl_x2;
gprMdl_y2=load(file_GPR2).gprMdl_y2;
gprMdl_z2=load(file_GPR2).gprMdl_z2;


if total_act(1)-total_act(2) == 3
    each_intent =1; last_intent = 2;
elseif total_act(1)-total_act(2) == -3
    each_intent =2; last_intent = 1;
end

        if each_intent == 1 
             temp_combine_X2=[each_test_file_add; train_X_xyz1;];
             temp_combine_Y2=[each_test_file_Y_add; train_Y_xyz1;];

              gprMdl_x2 = updateGPRMdl(gprMdl_x2, temp_combine_X2,  temp_combine_Y2(:,1));
              gprMdl_y2 = updateGPRMdl(gprMdl_y2, temp_combine_X2,  temp_combine_Y2(:,2));
              gprMdl_z2 = updateGPRMdl(gprMdl_z2, temp_combine_X2,  temp_combine_Y2(:,3));

        elseif each_intent == 2
             temp_combine_X1=[each_test_file_add; train_X_xyz2;];
             temp_combine_Y1=[each_test_file_Y_add; train_Y_xyz2;];          

 
              gprMdl_x1 = updateGPRMdl(gprMdl_x1, temp_combine_X1,  temp_combine_Y1(:,1));
              gprMdl_y1 = updateGPRMdl(gprMdl_y1, temp_combine_X1,  temp_combine_Y1(:,2));
              gprMdl_z1 = updateGPRMdl(gprMdl_z1, temp_combine_X1,  temp_combine_Y1(:,3));

        end

                
        result = myGRP_xyz(each_test_file_left(:,1:end-1), each_intent, new_gprMdl_1,new_gprMdl_2,new_gprMdl_3);

end