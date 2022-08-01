clear
clc
a=load('F:\Data\20201109-144702-woquan.txt');
for t=1:1:2000
        b(t,1)=a(t,1)*16*16+a(t,2);
%         b(t,1)=a(t,2)*16*16+a(t,3);
    c(t,1)=b(t,1)*3.3/4096;
end
x=c;
x=x';
%傅立叶变换
x=x-mean(x);                                               %消去直流分量，使频谱更能体现有效信息
Fs=1000;                %得到原始数据data.txt时，仪器的采样频率。就是length(x)/(max(x)-min(x));     
N=2000;                                                 %data.txt中的被测量个数，即采样个数。其实就是length(y);
z=fft(x);

%频谱分析
f=(0:N-1)*Fs/N;
Mag=2*abs(z)/N;                                        %幅值，单位同被测变量y
Mag=Mag./max(Mag);
Pyy=Mag.^2;          %能量；对实数系列X，有 X.*X=X.*conj(X)=abs(X).^2=X.^2，故这里有很多表达方式

%显示频谱图(频域)
subplot(2,1,2)
plot(f(1:N/2),Pyy(1:N/2),'r')                         %显示频谱图
%                 |
%             将这里的Pyy改成Mag就是 幅值-频率图了
axis([min(f(1:N/2)) max(f(1:N/2)) 1.1*floor(min(Pyy(1:N/2))) 1.1*ceil(max(Pyy(1:N/2)))]) 
xlabel('频率 (Hz)')
ylabel('能量')
title('频谱图(频域)')
grid on;

%返回最大能量对应的频率和周期值
[a b]=max(Pyy(1:N/2));
fprintf('\n傅立叶变换结果：\n') 
fprintf('           FFT_f = %1.3f Hz\n',f(b))             %输出最大值对应的频率
fprintf('           FFT_T = %1.3f s\n',1/f(b))          %输出最大值对应的周期