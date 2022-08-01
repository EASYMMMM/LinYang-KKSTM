close all;clear;clc;

% load arrhythmia
% tTree = templateTree('surrogate','on');
% tEnsemble = templateEnsemble('GentleBoost',100,tTree);
% options = statset('UseParallel',true);
% Mdl = fitcecoc(X,Y,'Coding','onevsall','Learners',tEnsemble,...
%                 'Prior','uniform','NumBins',50,'Options',options);
%  CVMdl = crossval(Mdl,'Options',options);      
%  
% oofLabel = kfoldPredict(CVMdl,'Options',options);
% ConfMat = confusionchart(Y,oofLabel,'RowSummary','total-normalized');
% % ConfMat.InnerPosition = [0.10 0.12 0.85 0.85];
 
load fisheriris
X = meas;
Y = categorical(species);
classOrder = unique(Y);
rng(1); % For reproducibility

t = templateSVM('Standardize',true);
PMdl = fitcecoc(X,Y,'Holdout',0.10,'Learners',t,'ClassNames',classOrder);
Mdl = PMdl.Trained{1};           % Extract trained, compact classifier

testInds = test(PMdl.Partition);  % Extract the test indices
XTest = X(testInds,:);
YTest = Y(testInds,:);
labels = predict(Mdl,XTest);

idx = randsample(sum(testInds),10);
table(YTest(idx),labels(idx),...
    'VariableNames',{'TrueLabels','PredictedLabels'})
