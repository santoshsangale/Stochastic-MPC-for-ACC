files = dir('Data*') ;  % you are in the folder of data files 
N = length(files) ; 
D = cell(N,1) ; 
for i = 1:N
    D{i} = load(files(i).name) ; 
end
D{1}.v_p_auswertung
