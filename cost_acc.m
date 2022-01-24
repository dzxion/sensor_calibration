function cost_function = cost_acc(acc_calib_params, static_samples, params_num)

cost_function = [];

if params_num == 9 % 9参数
    mis_mat = [1 , acc_calib_params(1),  acc_calib_params(2);
        0 ,  1   , acc_calib_params(3);
        0 ,  0   ,   1];
    scale_mat = [acc_calib_params(4) ,  0   ,  0;
        0    , acc_calib_params(5) ,  0;
        0    ,  0   , acc_calib_params(6)];
    bias_vec = [acc_calib_params(7);acc_calib_params(8);acc_calib_params(9)];
else if params_num == 12 % 12参数
        mis_mat = [1 , acc_calib_params(1), acc_calib_params(2);
            acc_calib_params(3) ,  1 , acc_calib_params(4);
            acc_calib_params(5) ,  acc_calib_params(6), 1];
        scale_mat = [acc_calib_params(7) ,  0   ,  0;
            0    , acc_calib_params(8) ,  0;
            0    ,  0   , acc_calib_params(9)];
        bias_vec = [acc_calib_params(10);acc_calib_params(11);acc_calib_params(12)];
    else
        
    end

for i=1:length(static_samples)
%     cost_function(i,1)=(1-(norm(mis_mat*scale_mat*(static_samples(:,i)+bias_vec))));
    cost_function(i,1)=(1365-(norm(mis_mat*scale_mat*(static_samples(:,i)+bias_vec))));
end

end