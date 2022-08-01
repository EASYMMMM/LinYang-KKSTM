clc;
clear all;
close all;
%%train and update
rng(0,'twister'); % For reproducibility
n = 1000;
x = linspace(-1000,1000,n)';
x_test = linspace(1001,3000,n)';
% x = 1:1000
g1=ones(1000,1);
g_1=-ones(1000,1);
y1 = 1 + sin(x./100)./x*100 + 0.2*randn(n,1)+x/100;
y_1 = 1  - sin(x./100)./x*100 + 0.2*randn(n,1)-x/100;
combine_x1=[x(1:500) ones(500,1)]; 
combine_x2=[x(1:500) -ones(500,1)]; 
combine_x3=[combine_x1; combine_x2;];
combine_x4=[[x ones(1000,1)];[x_test -ones(1000,1)];];

gprMdl_inter = fitrgp(combine_x4, [y1; y_1;]  ); % use half of the data for training
gprMdl_pn = fitrgp(combine_x3,[y1(1:500); y_1(1:500);] ); % use half of the data for training
gprMdl_nega = fitrgp(combine_x2, y_1(1:500) ); % use half of the data for training
gprMdl_no = fitrgp(x(1:500), y1(1:500) ); % use half of the data for training
gprMdl = fitrgp(combine_x1, y1(1:500) ); % use half of the data for training
% gprMdl2 = updateGPRMdl(gprMdl, x(1:end),y(1:end)); % now pass the entire data
% gprMdl2 = updateGPRMdl(gprMdl,[x(1:500); x(1:500);],[y1(1:500); zeros(500,1);]); % now pass the entire data


%%predict from the older mdl
[ypred, ~, yci] = predict(gprMdl_inter, [x ones(1000,1)] );

subplot(2,1,1);
title('no data, not much confidence on the right');
hold on;
plot([x; x_test;] ,gprMdl_inter.Y,'r.');
plot(x ,ypred);
plot(x, yci(:,1),'k:');
plot(x, yci(:,2),'k:');



% %%predict from the older mdl
% [ypred, ~, yci] = predict(gprMdl,[x g1*100]);
% 
% subplot(2,1,1);
% title('no data, not much confidence on the right');
% hold on;
% plot(x(1:500),gprMdl.Y,'r.');
% plot(x,ypred);
% plot(x,yci(:,1),'k:');
% plot(x,yci(:,2),'k:');


% %%predict from the older mdl
% [ypred, ~, yci] = predict(gprMdl_no,x);
% subplot(2,1,2);
% title('no data, not much confidence on the right');
% hold on;
% plot(x(1:500),gprMdl_no.Y,'r.');
% plot(x,ypred);
% plot(x,yci(:,1),'k:');
% plot(x,yci(:,2),'k:');



%%predict from the older mdl
[ypred, ~, yci] = predict(gprMdl_inter,  [x_test ones(1000,1)] );

subplot(2,1,2);
title('no data, not much confidence on the right');
hold on;
plot([x; x_test;] ,gprMdl_inter.Y,'r.');
plot(x ,ypred);
plot(x, yci(:,1),'k:');
plot(x, yci(:,2),'k:');


% %%predict from the updated mdl
% [ypred, ~, yci] = predict(gprMdl2,x);
% subplot(2,1,2);
% title('more confident prediction after adding data');
% hold on;
% plot(gprMdl2.X,gprMdl2.Y,'r.');
% plot(x,ypred);
% plot(x,yci(:,1),'k:');
% plot(x,yci(:,2),'k:');

% figure(11)
% jj=gprMdl.X;
% plot(jj,gprMdl.Y,'r.');

%%api to update mdl
function newmdl = updateGPRMdl(mdl,Xtrain, Ytrain) % a very simple updating api, assumes defaults, will need to be modified for nondefaults
kernelparams = mdl.KernelInformation.KernelParameters; 
inisigma = mdl.Sigma;
beta = mdl.Beta; 
newmdl = fitrgp(Xtrain, Ytrain, 'FitMethod', 'none', 'Sigma', inisigma, 'Beta', beta, 'KernelParameters', kernelparams); 
end