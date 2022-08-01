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
%����Ҷ�任
x=x-mean(x);                                               %��ȥֱ��������ʹƵ�׸���������Ч��Ϣ
Fs=1000;                %�õ�ԭʼ����data.txtʱ�������Ĳ���Ƶ�ʡ�����length(x)/(max(x)-min(x));     
N=2000;                                                 %data.txt�еı�������������������������ʵ����length(y);
z=fft(x);

%Ƶ�׷���
f=(0:N-1)*Fs/N;
Mag=2*abs(z)/N;                                        %��ֵ����λͬ�������y
Mag=Mag./max(Mag);
Pyy=Mag.^2;          %��������ʵ��ϵ��X���� X.*X=X.*conj(X)=abs(X).^2=X.^2���������кܶ��﷽ʽ

%��ʾƵ��ͼ(Ƶ��)
subplot(2,1,2)
plot(f(1:N/2),Pyy(1:N/2),'r')                         %��ʾƵ��ͼ
%                 |
%             �������Pyy�ĳ�Mag���� ��ֵ-Ƶ��ͼ��
axis([min(f(1:N/2)) max(f(1:N/2)) 1.1*floor(min(Pyy(1:N/2))) 1.1*ceil(max(Pyy(1:N/2)))]) 
xlabel('Ƶ�� (Hz)')
ylabel('����')
title('Ƶ��ͼ(Ƶ��)')
grid on;

%�������������Ӧ��Ƶ�ʺ�����ֵ
[a b]=max(Pyy(1:N/2));
fprintf('\n����Ҷ�任�����\n') 
fprintf('           FFT_f = %1.3f Hz\n',f(b))             %������ֵ��Ӧ��Ƶ��
fprintf('           FFT_T = %1.3f s\n',1/f(b))          %������ֵ��Ӧ������