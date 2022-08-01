close all;clear;clc;
%%train and update
rng(0,'twister'); % For reproducibility
n = 2000;
x = linspace(-10,10,n)';
y = 1 + x*5e-2 + sin(x)./x + 0.2*randn(n,1);
tic
gprMdl = fitrgp(x(1:500),y(1:500)); % use half of the data for training
toc

tic
gprMdl2 = updateGPRMdl(gprMdl, x,y); % now pass the entire data
toc

%%predict from the older mdl
[ypred, ~, yci] = predict(gprMdl,x);

subplot(2,1,1);
title('no data, not much confidence on the right');
hold on;
plot(gprMdl.Y,'r.');
plot(ypred);
plot(yci(:,1),'k:');
plot(yci(:,2),'k:');
%%predict from the updated mdl
[ypred, ~, yci] = predict(gprMdl2,x(501:600));

subplot(2,1,2);
title('more confident prediction after adding data');
hold on;
plot(gprMdl2.Y,'r.');
plot([501:600],ypred);
plot([501:600],yci(:,1),'k:');
plot([501:600],yci(:,2),'k:');

%%api to update mdl
function newmdl = updateGPRMdl(mdl,Xtrain, Ytrain) % a very simple updating api, assumes defaults, will need to be modified for nondefaults
kernelparams = mdl.KernelInformation.KernelParameters; 
inisigma = mdl.Sigma;
beta = mdl.Beta; 
newmdl = fitrgp(Xtrain, Ytrain, 'FitMethod', 'none', 'Sigma', inisigma, 'Beta', beta, 'KernelParameters', kernelparams); 
end