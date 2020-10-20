n = zeros(1, 10);
r = zeros(1, 10);
min_r = zeros(1, 10);
    
for k=1:10
    load(['models\np3\intersection\J', int2str(k),'.mat']);
    load(['summarized_results\logs\controllers\intersections\J', int2str(k),'.mat']);
    
    n(k) = size(matrix_c, 1);
    r_tmp = [];
    for t=2:length(ref_log)
      r_tmp = [r_tmp rank([matrix_q ref_log(t,:)'])];  
    end
    r(k) = rank(matrix_q);
    min_r(k) = min(r_tmp);
end

[n;r;min_r]