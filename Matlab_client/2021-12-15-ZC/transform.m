function after = transform(round_this) 

xyz=[];all_delta_theta=[];
 %% 坐标转换
 this_sliding=round_this(:,1:12);
        new_EMG2=[];
                delta_z=this_sliding(:,12)-this_sliding(:,3);
                delta_x=this_sliding(:,10)-this_sliding(:,1);
%                 delta_z=this_sliding(:,9);
%                 delta_x=this_sliding(:,7);
                delta_theta=atan(delta_z./delta_x);
                all_delta_theta=[all_delta_theta; delta_theta*180/pi;];
     for k = 1:1
            who_x=7;
            who_z=9;
            who_y=8;            

                rela_wrist_sh_x=this_sliding(:,who_x);
                rela_wrist_sh_z=this_sliding(:,who_z);
              
                    rela_wan_y=this_sliding(:,who_y);

                    output_this_silde_x=[];output_this_silde_z=[];
                    for each_in_silde = 1:size(this_sliding,1)
                        each_rela_jian_z=rela_wrist_sh_z(each_in_silde);
                        each_rela_jian_x=rela_wrist_sh_x(each_in_silde);
                        each_theta=45/180*pi;
%                         each_theta=delta_theta(each_in_silde);
%                         each_theta = pi/2-mean(delta_theta);
%                         after_x=cos(each_theta)*each_rela_jian_x+sin(each_theta)*each_rela_jian_z;
%                         after_z=cos(each_theta)*each_rela_jian_z-sin(each_theta)*each_rela_jian_x;
                        after_x=sin(each_theta)*each_rela_jian_x-cos(each_theta)*each_rela_jian_z;
                        after_z=cos(each_theta)*each_rela_jian_x+sin(each_theta)*each_rela_jian_z;                        
                        
                        output_this_silde_x=[output_this_silde_x; after_x;];
                        output_this_silde_z=[output_this_silde_z; after_z;];
                    end
                    new_EMG2=[ new_EMG2  output_this_silde_x rela_wan_y output_this_silde_z];    
     end
     after=[round_this(:,1:6) new_EMG2 round_this(:,10:end)];
%      new_EMG=[current_combine new_EMG2(:,13:15)-new_EMG2(:,19:21) new_EMG2(:,16:18)-new_EMG2(:,19:21)];
%      
%      all_new_EMG2=[all_new_EMG2; new_EMG2;];   
    
    
end