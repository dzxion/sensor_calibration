function data = unbiasNormalize_12(raw_data, calib_params)

data = [];
mis_mat = [1 , calib_params(1), calib_params(2);
    calib_params(3) ,  1 , calib_params(4);
    calib_params(5) ,  calib_params(6), 1];
scale_mat = [calib_params(7) ,  0   ,  0;
    0    , calib_params(8) ,  0;
    0    ,  0   , calib_params(9)];
bias_vec = [calib_params(10);calib_params(11);calib_params(12)];

for i=1:length(raw_data)
    cur_data = mis_mat*scale_mat*(raw_data(:,i)-bias_vec);
    data = [data,cur_data];
end

end