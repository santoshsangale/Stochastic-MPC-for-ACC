%D1=load('Data1');
% D2=load('Data2');
% D3=load('Data3');

% mu1=sum(D1.v_p_auswertung)/length(D1.v_p_auswertung);
% sigma1=std(D1.v_p_auswertung);
% m1=100;

% mu2=sum(D2.v_p_auswertung)/length(D2.v_p_auswertung);
% sigma2=std(D2.v_p_auswertung);
% m2=200;

% mu3=sum(D3.v_p_auswertung)/length(D3.v_p_auswertung);
% sigma3=std(D3.v_p_auswertung);
% m2=200;


% create GMM
mu = [0; 5; 20];        %[mu1; mu2]
%mu = [mu1; mu2;mu3];    
sigma = cat(3, 1, 2, 3);   %1 and 2 are variances
%sigma=cat(4,sigma1,sigma2,sigma3);
p = [0.5; 0.3;0.2];     %mixture proportion
gmm = gmdistribution(mu, sigma, p);

% view PDF
ezplot(@(x) pdf(gmm,x));