
figure;

for i=1:10;
    load(['intersections/J', int2str(i), '.mat']);
    N = length(cost_1_log);

    state_np1 = state_log(2:N-2,:);
    state_np2 = state_log(3:N-1,:);
    state_np3 = state_log(4:N,:);

    predict_np1 = state_predict.np0(1:N-3,:);
    predict_np2 = state_predict.np1(1:N-3,:);
    predict_np3 = state_predict.np2(1:N-3,:);

    error_np1_temp = state_np1 - predict_np1;
    error_np2_temp = state_np2 - predict_np2;
    error_np3_temp = state_np3 - predict_np3;

    error = zeros(3, N-3);

    for k=1:N-3;
        n = size(error_np1_temp(k,:), 2);
        error(1,k) = norm(error_np1_temp(k,:), 1)/n;
        error(2,k) = norm(error_np2_temp(k,:), 1)/n;
        error(3,k) = norm(error_np3_temp(k,:), 1)/n;
    end

    subplot(5,2, i);
    plot(1:N-3, error, 'LineWidth',2);
    title(['Intersection ', int2str(i)]);
    xlim([1, N-3]);
    
    if mod(i, 2) == 1;
        ylabel('MAE (Veh)');
    end
    
    if i == 9 || i == 10;
        xlabel('Cycle');
    end
    
    if i == 10;
        legend({'Np1','Np2','Np3'});
    end
end

