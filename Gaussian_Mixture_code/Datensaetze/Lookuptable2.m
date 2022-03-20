files = dir('Data*') ;  % you are in the folder of data files 
N = length(files) ; 
D = cell(N,1) ; 
for i = 1:N
    D{i} = load(files(i).name) ; 
end
%D{1}.v_p_auswertung;

% A1=D1.v_p_auswertung;A2=D2.v_p_auswertung;A3=D3.v_p_auswertung;
% A4=D4.v_p_auswertung;A5=D5.v_p_auswertung;A6=D6.v_p_auswertung;
% A7=D7.v_p_auswertung;A8=D8.v_p_auswertung;A9=D9.v_p_auswertung;
% A10=D10.v_p_auswertung;
r1=10;   %min value of velocity range m/s
r2=12;  %max value of velocity range in m/s
% 
% for j=1:87
%     A{j}=D{j}.v_p_auswertung;
% end

V={};
for i=1:87
    A{i}=D{i}.v_p_auswertung;
    V{i}=A{i}(A{i}>r1 & A{i}<=r2);
    
end

%%To find maximum size of V extracted
  for j=1:87
       findmax_V(j)=length(V{j});
    
     Max_length=max(findmax_V);
  end

% for k=1:87
%      mu{j}=(V{j}(1));
%     for i=1:1
%        mu1=sum(mu); 
%     end
% end

