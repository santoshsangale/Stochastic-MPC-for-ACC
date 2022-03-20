Final={};
for i = 1: 87
 a=load(['Data' num2str(i) '.mat']);
 vel=a.v_p_auswertung;
 Final{i}=vel(vel>10 &vel<=12);
    


end
[s,d] = cellfun(@size,Final);
out = max([s,d]);
V={};
for i =1:87

    for a=1:out
        if a<=length(Final{i})
            V{a}(1,i)=Final{1,i}(1,a);
        else
            V{a}(1,i)=nan;
        end
    end   
end
means={};
sigmas={};
for a=1:length(V)
    means{a}=nanmean(V{a});
    sigmas{a}=nanstd(V{a});
end

 mu =(cell2mat(means));        %mean of all 
 MU=mu';
% % %mu = [mu1; mu2;mu3];    
 %sigma = (cell2mat(sigmas));   %1 and 2 are variances
 
 
Sigma=sigma';
% Cov=zeros(782,782);
% p=zeros(782,1);
% p(1:1:end)=0.013;     %mixture proportion
%  gmm = gmdistribution(MU, Cov,p);
% % 
% % % view PDF
%  ezplot(@(x) pdf(gmm,x));

[M, S] = meshgrid(MU, Sigma    );
M = M(:);
S = S(:);
f =@(x) cell2mat(arrayfun(@(m,s) exp(-0.5*((x(:)' - m)./s).^2), M, S, 'uni', 0));
x = linspace(-50, 50, 1000);
plot(x, f(x))













