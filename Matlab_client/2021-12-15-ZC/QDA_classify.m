function [outclass, SigmaHat, logDetSigma,gmeans, posterior] = QDA_classify(sample, training, group, prior)
% 由MATLAB自带函数classify修改而来
[gindex,groups] = grp2idx(group);

ngroups = length(groups); %待分类的类别数,groups-->类别数  gindex-->每一个坐标对应的类别
gsize = hist(gindex,1:ngroups);%分配不同的盒子
nonemptygroups = find(gsize>0);

[n,d] = size(training);%行数和列数
m = size(sample,1);

if nargin < 4 || isempty(prior)    %如果没定义prior，则每一类的先验概率相同
    prior = ones(1, ngroups) / ngroups;
end

mm = m; %测试数据样本数

gmeans = NaN(ngroups, d);
for k = nonemptygroups
    gmeans(k,:) = mean(training(gindex==k,:),1);    %求每一类训练数据的均值
end

D = repmat(NaN, mm, ngroups);
logDetSigma = zeros(ngroups,1,class(training));
SigmaHat=zeros(d,d,ngroups);
R = zeros(d,d,ngroups);

% Pooled estimate of covariance.  Do not do pivoting, so that A can be
% computed without unpermuting.  Instead use SVD to find rank of R.
for k=nonemptygroups
    try
    [Q,R(:,:,k)] = qr(bsxfun(@minus,training(gindex==k,:),gmeans(k,:)), 0);
    catch
        training(gindex==k,:)
        gmeans(k,:)
    end
    R(:,:,k) = R(:,:,k) / sqrt(gsize(k)-1); % SigmaHat = R'*R， gsize：相应类别样本数
    SigmaHat(:,:,k) = R(:,:,k)'*R(:,:,k);
    s = svd(R(:,:,k));
    if any(s <= max(n,d) * eps(max(s)))

        error('stats:classify:BadVariance',...
              'The pooled covariance matrix of TRAINING must be positive definite.');
    end
    logDetSigma(k) = 2*sum(log(s)); % avoid over/underflow
end

% MVN relative log posterior density, by group, for each sample
for k = nonemptygroups
    A = (sample - repmat(gmeans(k,:), mm, 1)) / R(:,:,k);
    D(:,k) = log(prior(k)) - .5*(sum(A .* A, 2) + logDetSigma(k));
end


% find nearest group to each observation in sample data
[maxD,outclass] = max(D, [], 2);

% calculate the posterior probability
if nargout > 4   
    % Bayes' rule: first compute p{x,G_j} = p{x|G_j}Pr{G_j} ...
    % (scaled by max(p{x,G_j}) to avoid over/underflow)
    P = exp(D(1:m,:) - repmat(maxD(1:m),1,ngroups));
    sumP = nansum(P,2);
    % ... then Pr{G_j|x) = p(x,G_j} / sum(p(x,G_j}) ...
    % (numer and denom are both scaled, so it cancels out)
    posterior1 = P ./ repmat(sumP,1,ngroups);  
    posterior=max(posterior1,[],2);
end

% Convert back to original grouping variable type
if isnumeric(group)
   groups = str2num(char(groups));
   groups=cast(groups,class(group)); 
end

if isvector(groups)
    groups = groups(:);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
outclass = groups(outclass,:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

