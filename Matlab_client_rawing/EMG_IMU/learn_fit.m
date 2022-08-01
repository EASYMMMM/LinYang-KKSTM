%自定义拟合函数f(t)=a*cos(k*t)*exp(w*t)
clc,clear
syms t
% x=[0;0.4;1.2;2;2.8;3.6;4.4;5.2;6;7.2;8;9.2;10.4;11.6;12.4;13.6;14.4;15];%列向量
% y=ones(18,1);
% z=3*x.^3+y;

    
% cfun=fit([x y],z,g) %根据自定义拟合函数f来拟合数据x，y
% xi=0:0.1:20;
% yi=cfun(xi);
% plot(x,y,'r*',xi,yi,'b-');

load franke
sf = fit([x, y],z,'poly23')

g = fittype( @(a,b,c,d,x,y)  a*x.^2+c*x+b*y+d, ...
         'independent', {'x', 'y'}, ...
        'dependent', 'z' ); 
   
cfun=fit([x y],z,g) 

yi=cfun([x y])
figure(99)
plot(y); hold on; plot(yi)