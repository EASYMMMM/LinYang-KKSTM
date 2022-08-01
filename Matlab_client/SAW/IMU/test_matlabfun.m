close all;clear;clc;
% 这个用来测试意图，对人来说，1前 2后 3左 4右 5 上 6下 7不知道
all_std=[];
tic
for file =1:4
    if file == 1
        load('20220414_testleft.mat');
        
    elseif file ==2
        load('20220414_testforw.mat');
    elseif file ==3
        load('20220414_testup.mat');
    elseif file ==4
        load('20220418_ful2.mat');      
        ori_angles2=ori_angles.';
    end
 all_std=[];   
intent=[];
all_eul12=[];all_eul13=[];all_eul23=[];
all_eul2=[];all_eul1=[];all_eul3=[];
all_w1=[];all_w2=[];all_w3=[];
all_w12=[];all_w23=[];all_w13=[];
all_y12=[];all_a12=[];all_b12=[];
all_y23=[];all_a23=[];all_b23=[];
all_y13=[];all_a13=[];all_b13=[];
w13=zeros(3,3); w23=zeros(3,3);  w12=zeros(3,3); 
last_wx=0; last_wy=0; last_wz=0;

plus=0;

which1=max(find(ori_angles2(:,1)==1))+1
ori_angles3=ori_angles2(which1:end,:);
% ori_angles3=ori_angles2(1:end,:);
last_eu13=ori_angles3(1,3)*pi/180;last_eu33=ori_angles3(1,9)*pi/180;

for k = which1:size(ori_angles2,1)
    
    t1=data_all_IMU(2,k);
    t2=data_all_IMU(2+25,k);
    t3=data_all_IMU(2+50,k);
    eul1=ori_angles2(k,1:3)*pi/180;
    eul2=ori_angles2(k,4:6)*pi/180;
    eul3=ori_angles2(k,7:9)*pi/180;


    
   if (eul1(3)<150*pi/180 && eul1(3)>-150*pi/180)
    if abs(eul1(3)-last_eu13) > 40*pi/180 
        eul1(3) = last_eu13;
    end
   end
   
   if (eul3(3)<150*pi/180 && eul3(3)>-150*pi/180)
    if abs(eul3(3)-last_eu33) > 40*pi/180 
        eul3(3) = last_eu33;
    end
   end

    
    all_eul1=[all_eul1; eul1;];
    all_eul3=[all_eul3; eul3;];
    all_eul2=[all_eul2; eul2;];
    
    last_eu13=eul1(3);last_eu33=eul3(3);   
    
    
    eul3=fliplr(eul3); eul2=fliplr(eul2); eul1=fliplr(eul1);
    R1=eul2rotm(eul1);
    R2=eul2rotm(eul2);
    R3=eul2rotm(eul3);
    if k>which1
        w1=(R1-lastR1)/(t1-last_t1)*R1';
        w2=(R2-lastR2)/(t2-last_t2)*R2';
        w3=(R3-lastR3)/(t3-last_t3)*R3';
        all_w1=[all_w1 [w1(3,2); -w1(3,1); w1(2,1);]];
        all_w2=[all_w2 [w2(3,2); -w2(3,1); w2(2,1);]];
        all_w3=[all_w3 [w3(3,2); -w3(3,1); w3(2,1);]];
    end
    lastR1=R1;
    lastR2=R2;
    lastR3=R3;
    eul1 = rotm2eul(R1)*180/pi;
    eul3 = rotm2eul(R3)*180/pi;
    eul2 = rotm2eul(R2)*180/pi;
%     all_eul1=[all_eul1; eul1;];
%     all_eul3=[all_eul3; eul3;];
%     all_eul2=[all_eul2; eul2;];
    R13 = R3'*R1;
    R23 = R3'*R2;
    R12 = R2'*R1;
    if k>which1
        if t1 ~= last_t1
            w13=(R13-lastR13)/(t1-last_t1)*R13';
        end
        if t2 ~= last_t2
            w23=(R23-lastR23)/(t2-last_t2)*R23';
        end
        if t3 ~= last_t3
            w12=(R12-lastR12)/(t3-last_t3)*R12';
        end   
        
% 1前 2后 3左 4右 5上 6下 7不知道      
        w_xyz=[w13(3,2); -w13(3,1); w13(2,1);];
        w_xyz23=[w23(3,2); -w23(3,1); w23(2,1);];
%         if file == 2
            all_std=[all_std std(w_xyz)/mean(w_xyz)];
            
        wx23=w_xyz23(1); wy23=w_xyz23(2); wz23=w_xyz23(3);
        wx=w_xyz(1); wy=w_xyz(2); wz=w_xyz(3);
%         if abs(wx-last_wx)>0.5
%             wx=last_wx;
%         else
%             last_wx=wx;
%         end
%         if abs(wy-last_wy)>0.5
%             wy=last_wy;
%         else
%             last_wy=wy;
%         end
%         if abs(wz-last_wz)>0.5
%             wz=last_wz;
%         else
%             last_wz=wz;
%         end
% w_xyz=[wx; wy;wz];     
%% classify by human        
if wx23>0 && wy23>0 && wz23>0 && sum(abs(w_xyz23)>0.2) > 0 && max(w_xyz23)/median(w_xyz23)<2.2
    intent=[intent 1];
elseif wx23<0 && wy23<0 && wz23<0 && sum(abs(w_xyz23)>0.2) > 0 && min(w_xyz23)/median(w_xyz23)<2.2
    intent=[intent 2];
elseif abs(wy-wx)/abs(wx) > abs(wy-wx)/abs(wy) && abs(wy-wx)/abs(wx) > 2  && sum(abs(w_xyz)>0.1) > 0 
    
    if wy > 0
        intent=[intent 4];
    else
        intent=[intent 3];
    end
    
elseif abs(wy-wx)/abs(wx) < abs(wy-wx)/abs(wy) && abs(wy-wx)/abs(wy) > 2  && sum(abs(w_xyz)>0.2) > 0
    
    if wx > 0
        intent=[intent 5];
    else
        intent=[intent 6];
    end
elseif abs(wz-wx)/abs(wx) > 1.5  && sum(abs(w_xyz)>0.2) > 0
    if wz > 0
        intent=[intent 3];
    else
        intent=[intent 4];
    end
else
    
    
    intent=[intent 7];
    
end
        
        all_w13=[all_w13 w_xyz];
        all_w23=[all_w23 [w23(3,2); -w23(3,1); w23(2,1);]];
        all_w12=[all_w12 [w12(3,2); -w12(3,1); w12(2,1);]];
    end
    lastR13=R13;
    lastR23=R23;
    lastR12=R12;    
    
    eul12 = rotm2eul(R12)*180/pi;
    eul13 = rotm2eul(R13,'XYZ')*180/pi;
    eul23 = rotm2eul(R23)*180/pi;
    all_eul12=[all_eul12; eul12;];
    all_eul13=[all_eul13; eul13;];
    all_eul23=[all_eul23; eul23;];
%     if k == 1
        last_t1=t1;
        last_t2=t2;
        last_t3=t3;
%     else
%         if last_t1~=t1
%             last_t1=t1
%         end
%         if last_t2~=t2
%             last_t2=t2
%         end
%         if last_t3~=t3
%             last_t3=t3
%         end
%     end


end
figure(file);
plot(all_w13');hold on;
plot(intent);

legend('wx','wy','wz','intent')

if file ==2
  figure(45)
plot(all_w13');hold on;
plot(all_std);
end

if file < 4
figure(10)
plot(all_std); hold on;
jj=toc
end
end




% figure(1);
% plot(all_eul1);
% figure(2);
% plot(all_eul2);
% figure(3);
% plot(all_eul3);


figure(12);
plot(all_w12');
figure(13);
plot(all_w13');
legend('wx','wy','wz')
figure(23);
plot(all_w23');

% figure(31)
% plot(all_w1');
% figure(32)
% plot(all_w2');
% figure(33)
% plot(all_w3');
% 
% % figure(31)
% % plot(all_w1');
% % figure(22)
% % plot(all_w2');
% % figure(23)
% % plot(all_w3');
% 
% 
% figure(61)
% plot(all_eul12(:,3)); hold on;
% plot(all_eul23(:,3));
% legend('1x','2x')




