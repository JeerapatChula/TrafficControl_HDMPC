
error1 = zeros(10,1);
error2 = zeros(10,1);

for J=1:10
    temp1 = load(['intersections/J',int2str(J),'.mat']);
    error = abs(temp1.state_log(2:end,:) - temp1.state_predict.np0(1:end-1,:));
    error1(J) = sum(sum(error))/(size(error, 1)*size(error, 2));

    temp2 = load(['intersections_testmodel/J',int2str(J),'.mat']);
    phase_list = fieldnames(temp2.log_predict_x);
    log_predict_x = eval(['temp2.log_predict_x.', phase_list{end}]);
    error = abs(temp2.log_x(2:end,:) - double(log_predict_x(1:end-1,:)));
    error2(J) = sum(sum(error))/(size(error, 1)*size(error, 2));
end

%bar(x, error1,'stacked'); hold;
%bar(x, error2,'stacked');

bar([error2 error1],'stacked');
legend('Piecewise Model', 'Linear Model');
ylabel('Mean Absolute Error (veh/lane)');
xlabel('Intersection');


temp1 = sum(error1);
temp2 = sum(error2);
diff = (temp2-temp1)/(temp1+temp2)