function cost_function = cost_gyro(gyro_calib_params, static_samples)

cost_function = [];

mis_mat = [1 , gyro_calib_params(1), gyro_calib_params(2);
    gyro_calib_params(3) ,  1 , gyro_calib_params(4);
    gyro_calib_params(5) ,  gyro_calib_params(6), 1];
scale_mat = [gyro_calib_params(7) ,  0   ,  0;
    0    , gyro_calib_params(8) ,  0;
    0    ,  0   , gyro_calib_params(9)];
bias_vec = [gyro_calib_params(10);gyro_calib_params(11);gyro_calib_params(12)];

for i=1:length(static_samples)
    cost_function((i-1)*3+1) = diff(1);
    cost_function((i-1)*3+2) = diff(2);
    cost_function((i-1)*3+3) = diff(3);
end

end