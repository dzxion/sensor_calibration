function data = unbiasNormalize_9(raw_data, calib_params)

data = [];
mis_mat = [1 , calib_params(1), calib_params(2);
    0 ,  1 , calib_params(3);
    0 ,  0, 1];
scale_mat = [calib_params(4) ,  0   ,  0;
    0    , calib_params(5) ,  0;
    0    ,  0   , calib_params(6)];
bias_vec = [calib_params(7);calib_params(8);calib_params(9)];

for i=1:length(raw_data)
    cur_data = mis_mat*scale_mat*(raw_data(:,i)-bias_vec);
    data = [data,cur_data];
end

end