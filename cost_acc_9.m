function cost_function = cost_acc_9(acc_calib_params, static_samples)

cost_function = [];

mis_mat = [1 , acc_calib_params(1),  acc_calib_params(2);
    0 ,  1   , acc_calib_params(3);
    0 ,  0   ,   1];
scale_mat = [acc_calib_params(4) ,  0   ,  0;
    0    , acc_calib_params(5) ,  0;
    0    ,  0   , acc_calib_params(6)];
bias_vec = [acc_calib_params(7);acc_calib_params(8);acc_calib_params(9)];

for i=1:length(static_samples)
    cost_function(i,1)=(1365-(norm(mis_mat*scale_mat*(static_samples(:,i)+bias_vec))));
end

end