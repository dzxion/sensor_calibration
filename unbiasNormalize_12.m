function data = unbiasNormalize_12(raw_data, acc_calib_params)

data = [];
mis_mat = [1 , acc_calib_params(1), acc_calib_params(2);
    acc_calib_params(3) ,  1 , acc_calib_params(4);
    acc_calib_params(5) ,  acc_calib_params(6), 1];
scale_mat = [acc_calib_params(7) ,  0   ,  0;
    0    , acc_calib_params(8) ,  0;
    0    ,  0   , acc_calib_params(9)];
bias_vec = [acc_calib_params(10);acc_calib_params(11);acc_calib_params(12)];

for i=1:length(raw_data)
    cur_data = mis_mat*scale_mat*(raw_data(:,i)+bias_vec);
    data = [data,cur_data];
end

end