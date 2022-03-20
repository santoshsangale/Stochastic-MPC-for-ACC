D1=load('Data1');D2=load('Data2');D3=load('Data3');D4=load('Data4');
D5=load('Data5');D6=load('Data6');D7=load('Data7');D8=load('Data8');
D9=load('Data9');D10=load('Data10');
D11=load('Data11');D12=load('Data12');D13=load('Data13');D14=load('Data14');
D15=load('Data15');D16=load('Data16');D17=load('Data17');D18=load('Data18');
D19=load('Data19');D20=load('Data20');D21=load('Data21.mat');D22=load('Data22.mat');
D23=load('Data23.mat');D24=load('Data24.mat');D25=load('Data25.mat');
D26=load('Data26.mat');D27=load('Data27.mat');D28=load('Data28.mat');
D29=load('Data29.mat');D30=load('Data30.mat');D31=load('Data31.mat');
D32=load('Data32.mat');D33=load('Data33.mat');D34=load('Data34.mat');
D35=load('Data35.mat');D36=load('Data36.mat');D37=load('Data37.mat');
D38=load('Data38.mat');D39=load('Data39.mat');D40=load('Data40.mat');
D41=load('Data41.mat');D42=load('Data42.mat');D43=load('Data43.mat');
D44=load('Data44.mat');D45=load('Data45.mat');D46=load('Data46.mat');
D47=load('Data47.mat');D48=load('Data48.mat');D49=load('Data49.mat');
D50=load('Data50.mat');D51=load('Data51.mat');D52=load('Data52.mat');
D53=load('Data53.mat');D54=load('Data54.mat');D55=load('Data55.mat');
D56=load('Data56.mat');D57=load('Data57.mat');D58=load('Data58.mat');
D59=load('Data59.mat');D60=load('Data60.mat');D61=load('Data61.mat');
D62=load('Data62.mat');D63=load('Data63.mat');D64=load('Data64.mat');
D65=load('Data65.mat');D66=load('Data66.mat');D67=load('Data67.mat');
D68=load('Data68.mat');D69=load('Data69.mat');D70=load('Data70.mat');
D71=load('Data71.mat');D72=load('Data72.mat');D73=load('Data73.mat');
D74=load('Data74.mat');D75=load('Data75.mat');D76=load('Data76.mat');
D77=load('Data77.mat');D78=load('Data78.mat');D79=load('Data79.mat');
D80=load('Data80.mat');D81=load('Data81.mat');D82=load('Data82.mat');
D83=load('Data83.mat');D84=load('Data84.mat');D85=load('Data85.mat');
D86=load('Data86.mat');D87=load('Data87.mat');



%Range=[10,15];
%Lookup table
A1=D1.v_p_auswertung;A2=D2.v_p_auswertung;A3=D3.v_p_auswertung;
A4=D4.v_p_auswertung;A5=D5.v_p_auswertung;A6=D6.v_p_auswertung;
A7=D7.v_p_auswertung;A8=D8.v_p_auswertung;A9=D9.v_p_auswertung;
A10=D10.v_p_auswertung;
r1=10;   %min value of velocity range m/s
r2=15;  %max value of velocity range in m/s
%at step k1
R1 = A1(A1>r1 & A1<=r2);  %all the elements of Data1 within range r1 and r2
%length(R1) %number of elements of A1 between r1 and r2
R2 = A2(A2>r1 & A2<=r2);
R3 = A3(A3>r1 & A3<=r2); 
R4 = A1(A4>r1 & A4<=r2); 
R5 = A5(A5>r1 & A5<=r2); 
R6 = A6(A6>r1 & A6<=r2); 
R7 = A7(A7>r1 & A7<=r2); 
R8 = A8(A8>r1 & A8<=r2); 
R9 = A9(A9>r1 & A9<=r2);
R10 = A10(A10>r1 & A10<=r2);
%R=zeros(87,size(D));
% for i=1:87
%     R(i)=A(i)(A(i)>r1 & A(i)<=r2)
% end
a1=R1(1);a2=R2(1);a3=R3(1);a4=R4(1);a5=R5(1);a6=R6(1);a7=R7(1);a8=R8(1);a9=R9(1);a10=R10(1);
mu1=(a1+a2+a3+a4+a5+a6+a7+a8+a9+a10)/10;
mu2=10.55;
sigma1=1;
sigma2=2;
sigma=cat(3,2,2);
%mu=[mu1; mu2];

%p = [0.5; 0.5];     %mixture proportion
%gmm = gmdistribution(mu, sigma, p);

% view PDF
%ezplot(@(x) pdf(gmm,x));


% pd2 = makedist('Normal','mu',mu1,'sigma',1);
% pd3 = makedist('Normal','mu',mu2,'sigma',2);
% %pd5 = makedist('Normal','mu',5,'sigma',0.5);
% x = 0 : 0.001 : 100;
% y = 0.5*pdf(pd2,x) + 0.5*pdf(pd3,x);
% figure
% plot(x,y)
% 


