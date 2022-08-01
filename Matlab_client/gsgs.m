
window = zeros(100,7); 
for i = 1:100
    fuck = ones(1,7)*i;
    window(i,:)=fuck;
%     window = [new(i - 99 : i - 1,:); all_jtor1(i,:)];
%     temp = mean(window);
%     new = [new; temp];
end

% for q = 1:length(all_jtor1)-4
%     this = all_jtor1(q+4);
%     avg=(sum(new(q:q+3))+this)/5;
%     new=[new; avg;];
% %     break
% end
x2=all_F_contact(2,1:end);
Hd=lowpass_FIR_order2;
x=filter(Hd,x2);

figure(2)
plot(x2);hold on;plot(x);

% 
% 
% %傅立叶变换
% x=x-mean(x);                                               %消去直流分量，使频谱更能体现有效信息
% Fs=100;                 %得到原始数据data.txt时，仪器的采样频率。就是length(x)/(max(x)-min(x));     
% N=length(x);                                                 %data.txt中的被测量个数，即采样个数。其实就是length(y);
% z=fft(x);
% 
% %频谱分析
% f=(0:N-1)*Fs/N;
% Mag=2*abs(z)/N;                                        %幅值，单位同被测变量y
% Mag=Mag./max(Mag);
% Pyy=Mag.^2;          %能量；对实数系列X，有 X.*X=X.*conj(X)=abs(X).^2=X.^2，故这里有很多表达方式
% 
% %显示频谱图(频域)
% subplot(2,1,2)
% plot(f(1:N/2),Pyy(1:N/2),'r')                         %显示频谱图
% %                 |
% %             将这里的Pyy改成Mag就是 幅值-频率图了
% axis([min(f(1:N/2)) max(f(1:N/2)) 1.1*floor(min(Pyy(1:N/2))) 1.1*ceil(max(Pyy(1:N/2)))]) 
% xlabel('频率 (Hz)')
% ylabel('能量')
% title('频谱图(频域)')
% grid on;
% 
% %返回最大能量对应的频率和周期值
% [a b]=max(Pyy(1:N/2));
% fprintf('\n傅立叶变换结果：\n') 
% fprintf('           FFT_f = %1.3f Hz\n',f(b))             %输出最大值对应的频率
% fprintf('           FFT_T = %1.3f s\n',1/f(b))          %输出最大值对应的周期
% 
% % my_v=all_v_cartesian(2,2:end);
% % last=my_v(1);new_v=[last];
% % for q =2:length(my_v)
% %     this=my_v(q)
% %     avg=(0.02*this+0.98*last);
% %     new_v=[new_v avg];
% %     last=new_v(end);
% % end
% % plot(my_v);hold on; plot(new_v);
% coef=[
%      0.0369640968358,  0.03731986366053,   0.0376474852095,  0.03794657332815,...
%     0.03821677349033,  0.03845776526829,  0.03866926275761,  0.03885101495644,...
%     0.03900280609844,   0.0391244559391,  0.03921581999492,  0.03927678973513,...
%     0.03930729272575,  0.03930729272575,  0.03927678973513,  0.03921581999492,...
%      0.0391244559391,  0.03900280609844,  0.03885101495644,  0.03866926275761,...
%     0.03845776526829,  0.03821677349033,  0.03794657332815,   0.0376474852095,...
%     0.03731986366053,   0.0369640968358
% ].';
% coef_v_order9=[
%     0.09637703954237,  0.09877664875597,   0.1005959487483,   0.1018182036649,...
%      0.1024321592885,   0.1024321592885,   0.1018182036649,   0.1005959487483,...
%     0.09877664875597,  0.09637703954237
% ].';
% 
% my_v=all_v_cartesian(2,1:end);
% last=my_v(1:9);new_v=[last];
% for q =10:length(my_v)
%     this=my_v(q);
%     this_window=[last this];
%     avg=this_window*coef;
%     new_v=[new_v avg];
%     last=new_v(end-8:end);
% end
% figure(10)
% plot(my_v);hold on; plot(new_v);




