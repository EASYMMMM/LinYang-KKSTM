function newmdl = updateGPRMdl(mdl,Xtrain, Ytrain) % a very simple updating api, assumes defaults, will need to be modified for nondefaults
kernelparams = mdl.KernelInformation.KernelParameters; 
inisigma = mdl.Sigma;
beta = mdl.Beta; 
newmdl = fitrgp(Xtrain, Ytrain, 'FitMethod', 'none', 'Sigma', inisigma, 'Beta', beta, 'KernelParameters', kernelparams); 
end