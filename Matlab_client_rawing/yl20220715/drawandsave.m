function [data_head,total_rece_1,total_rece_2,total_rece_3,total_rece_4,total_rece_5,total_rece_6,total_rece,out_init] = drawandsave(flag_main,q,input_data_head,total_rece_1,total_rece_2,total_rece_3,total_rece_4,total_rece_5,total_rece_6,total_rece,input_init)
% 
% global b emg_show handle flag
% flag = 0;
init=input_init;
%% 

    if mod(flag_main,10000) == 0
         clust = parcluster('local');
         job1 = createJob(clust); %开启一个job
         disp('saving------');
         temp = total_rece;
         createTask(job1,@mytxt,1,{temp});%再给job1分配一个‘mytxt’的task
         submit(job1);
         total_rece = [];
    end
    pause(0.001);
    data_head=input_data_head;
   if data_head >= 50;
       drawfigure1(total_rece_1,total_rece_2,total_rece_3,total_rece_4,total_rece_5,total_rece_6,data_head,init);
       init=1;
     data_head=0;
     total_rece_0=[];total_rece_1=[];total_rece_2=[];total_rece_3=[];total_rece_4=[];total_rece_5=[];total_rece_6=[];
   end

        total_rece_6 = [total_rece_6 q(6)];
        total_rece_1 = [total_rece_1 q(1)];
        total_rece_2 = [total_rece_2 q(2)];
        total_rece_3 = [total_rece_3 q(3)];
        total_rece_4 = [total_rece_4 q(4)];
        total_rece_5 = [total_rece_5 q(5)];         
        
    total_rece = [total_rece q(1)];
    total_rece = [total_rece q(2)];
    total_rece = [total_rece q(3)];
    total_rece = [total_rece q(4)];
    total_rece = [total_rece q(5)];
    total_rece = [total_rece q(6)];

    data_head=data_head+1;

out_init=init;


function drawfigure1(total_rece_0,total_rece_1,total_rece_2,total_rece_3,total_rece_4,total_rece_5,draw_points,init) 
global b1 emg_show1 handle1 b2 emg_show2 handle2 b3 emg_show3 handle3 b4 emg_show4 handle4 b5 emg_show5 handle5 b6 emg_show6 handle6  
    if init == 0
        init = 1
        emg_show1  = zeros(5000,1);
        emg_show2  = zeros(5000,1);
        emg_show3  = zeros(5000,1);
        emg_show4  = zeros(5000,1);
        emg_show5  = zeros(5000,1);
        emg_show6  = zeros(5000,1);
        
        figure(100);
        subplot(3,2,1);
        handle1 = plot(b1);
        xlabel('joint1')
        ylabel('rad')
        subplot(3,2,2);
        handle2 = plot(b2);
        xlabel('joint2')
        ylabel('rad')
        subplot(3,2,3);
        handle3 = plot(b3);
        xlabel('joint3')
        ylabel('rad')
        subplot(3,2,4);
        handle4 = plot(b4);
        xlabel('joint4')
        ylabel('rad')
        subplot(3,2,5);
        handle5 = plot(b5);
        xlabel('joint5')
        ylabel('rad')
        subplot(3,2,6);
        handle6 = plot(b6);
        xlabel('joint6')
        ylabel('rad')
        drawnow;
        
    else
        b1=total_rece_0(end-draw_points+1:end);
  
        emg_show1(1:(end-length(b1)))=emg_show1((length(b1)+1):end);
        emg_show1((end-length(b1)+1):end)=b1;
        set(handle1,'ydata',emg_show1);
        
        b2=total_rece_1(end-draw_points+1:end);
        emg_show2(1:(end-length(b2)))=emg_show2((length(b2)+1):end);
        emg_show2((end-length(b2)+1):end)=b2;
        set(handle2,'ydata',emg_show2);
        
        b3=total_rece_2(end-draw_points+1:end);
        emg_show3(1:(end-length(b3)))=emg_show3((length(b3)+1):end);
        emg_show3((end-length(b3)+1):end)=b3;
        set(handle3,'ydata',emg_show3);
        
        b4=total_rece_3(end-draw_points+1:end);
        emg_show4(1:(end-length(b4)))=emg_show4((length(b4)+1):end);
        emg_show4((end-length(b4)+1):end)=b4;
        set(handle4,'ydata',emg_show4);
        
        b5=total_rece_4(end-draw_points+1:end);
        emg_show5(1:(end-length(b5)))=emg_show5((length(b5)+1):end);
        emg_show5((end-length(b5)+1):end)=b5;
        set(handle5,'ydata',emg_show5);
        
        b6=total_rece_5(end-draw_points+1:end);
        emg_show6(1:(end-length(b6)))=emg_show6((length(b6)+1):end);
        emg_show6((end-length(b6)+1):end)=b6;
        set(handle6,'ydata',emg_show6);
        drawnow;
    end
end

end