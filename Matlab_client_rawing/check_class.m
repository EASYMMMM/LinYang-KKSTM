close all;clear;clc;
load('20220123notveryfail.mat')

ooh=data_all(:,find(data_all(1,:) ~= 0));

map_EMG=ooh(2:end-1,:);

predict_test_y=[];predict_test_y_KKK=[]; count_up=0;output=[];


%% Initalize the coefficient of LASSO and judge 
file='ylcoef_0123.mat';
coef_4=load(file).coef_4;
coef_2=load(file).coef_2;
fitinfo2=load(file).fitinfo2;
fitinfo4=load(file).fitinfo4;
coef_3=load(file).coef_3;
fitinfo3=load(file).fitinfo3;
Mdl_1=load(file).Mdl_1;
Mdl=load(file).Mdl;



for inside =1:size(map_EMG,2)
    inside
    this_frame_EMG=[map_EMG(2:end,inside)].';
    EMG1=map_EMG(1,inside);
    pre_z=this_frame_EMG*coef_4+fitinfo4.Intercept;
    all_pre_z=[all_pre_z pre_z];
    if EMG1 > 0.2
        outclass = predict(Mdl,this_frame_EMG);
        if outclass == 1
            output=[output 2];
            
            
        elseif outclass == 2
            output=[output 2];
            
        elseif outclass == 3
            output=[output 3];
            
        elseif outclass == 4
            output=[output 4];
            
        else
            output=[output 0];
            
        end
        if length(output) >=11
            how3=length(find(output(end-10:end)==3));
            how2=length(find(output(end-10:end)==2));
            if how3>how2
                pre_y=(this_frame_EMG*coef_3+fitinfo3.Intercept)*how3/10;
            else
                pre_y=(this_frame_EMG*coef_2+fitinfo2.Intercept)*how2/10;
            end
        else
            pre_y=0;
        end
        
        
        
        predict_F=[predict_F pre_y];
        F_filtered2 = kalmanF(pre_y);
        predict_test_y_KKK=[predict_test_y_KKK; F_filtered2;];
        F_filtered2=0;
        
        
    else
        output=[output -1];
        STOP=STOP+1;
        disp('EMG nonooo')
    end
    
end
        


close all
    figure(45);
    
    plot(predict_test_y_KKK); hold on;
    plot(output); 
    legend('预测','out')