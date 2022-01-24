function data = unbiasNormalize_9(raw_data, acc_calib_params)

data = [];
mis_mat = [1 , acc_calib_params(1), acc_calib_params(2);
    0 ,  1 , acc_calib_params(3);
    0 ,  0, 1];
scale_mat = [acc_calib_params(4) ,  0   ,  0;
    0    , acc_calib_params(5) ,  0;
    0    ,  0   , acc_calib_params(6)];
bias_vec = [acc_calib_params(7);acc_calib_params(8);acc_calib_params(9)];

for i=1:length(raw_data)
    cur_data = mis_mat*scale_mat*(raw_data(:,i)+bias_vec);
    data = [data,cur_data];
end

end