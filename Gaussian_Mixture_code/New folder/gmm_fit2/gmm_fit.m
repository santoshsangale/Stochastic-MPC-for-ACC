function [mu,sigma,weight,mAIC,rr]=gmm_fit(sdata,N)
K=150;
xlow=min(sdata);
xhigh=max(sdata);
xaxis = [xlow:range(sdata)/(K-1):xhigh]'; 
xpdf  = hist(sdata,K);
xpdf  = xpdf/(sum(xpdf));
figure(1)
bar(xaxis, xpdf,'FaceColor','none');
hold on 
mAIC=[];
mu     = zeros(N,1);
sigma  = zeros(N,1);
weight = zeros(N,1);
gaussPdfi = zeros(K,N);
gaussPdf  = zeros(K,1);
options = statset('MaxIter',1000,'Display','final','TolFun',1e-6);
obj = gmdistribution.fit(sdata,N,'Options',options,'CovarianceType','diagonal');
mAIC=obj.AIC;
gaussPdf = pdf(obj,xaxis);
A = sum(gaussPdf);
gaussPdf = gaussPdf/A;
% separating N Gaussians
for n = 1:N,
    mu(n)          = obj.mu(n);
    sigma(n)       = sqrt(obj.Sigma(1,1,n));
    weight(n)      = obj.PComponents(n);
    gaussPdfi(:,n) = weight(n)*normpdf(xaxis,mu(n),sigma(n))/A;
end
%figure(2)
set(gcf, 'color', 'w');
set(gca, 'fontweight','bold','FontSize', 12);
plot(xaxis, gaussPdf, 'k', 'linewidth', 3);
xlabel('Data','fontweight','bold','Fontsize', 14);
ylabel('Density ','fontweight','bold', 'Fontsize', 14);
axis tight;
h=legend('Actual data','GM-fit');
set(h,'fontsize',12,'fontweight','b') 
rr=rmse(xpdf,gaussPdf');
rect = [0.25, 0.25, .25, .25];
set(h, 'Position', rect)
hold off;
end
