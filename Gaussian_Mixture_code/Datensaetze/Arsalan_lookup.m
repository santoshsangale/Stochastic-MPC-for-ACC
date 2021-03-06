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
for a=1:length(V)
    means{a}=nanmean(V{a});
end

mu =cell2mat(means);        %mean of all 
%mu = [mu1; mu2;mu3];    
sigma = ones(1,782);   %1 and 2 are variances
%sigma=cat(4,sigma1,sigma2,sigma3);
%p = [0.5; 0.3;0.2];     %mixture proportion
gmm = gmdistribution(mu, sigma);

% view PDF
ezplot(@(x) pdf(gmm,x));

