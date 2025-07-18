function cost_function = cost_gyro(gyro_calib_params, gyro_samples, dt, extracted_samples, extracted_intervals, sensitivity)
% input:
% gyro_calib_params - 待优化的陀螺参数
% gyro_samples - 陀螺原始数据（去零偏）
% dt - 角速度采样时间
% extracted_samples - 重力静态数据（校准后的）
% extracted_intervals - 静态区间
% sensitivity - 陀螺灵敏度（转化为弧度）
%
% output:
% cost_function - 代价函数（向量）

cost_function = [];
mis_mat = [1 , gyro_calib_params(1), gyro_calib_params(2);
    gyro_calib_params(3) ,  1 , gyro_calib_params(4);
    gyro_calib_params(5) ,  gyro_calib_params(6), 1];
scale_mat = [gyro_calib_params(7) ,  0   ,  0;
    0    , gyro_calib_params(8) ,  0;
    0    ,  0   , gyro_calib_params(9)];
bias_vec = [0;0;0];

% 遍历所有运动区间
for i=1:length(extracted_samples)-1
    g_versor_pos0 = extracted_samples(1:3,i);
    g_versor_pos1 = extracted_samples(1:3,i+1);
    g_versor_pos0 = g_versor_pos0 / norm(g_versor_pos0);
    g_versor_pos1 = g_versor_pos1 / norm(g_versor_pos1);
    gyro_interval.start_idx = extracted_intervals(i).end_idx;
    gyro_interval.end_idx = extracted_intervals(i+1).start_idx;
    calib_gyro_samples = [];
    % 计算带优化参数的陀螺数据
    for i=gyro_interval.start_idx:gyro_interval.end_idx
        cur_calib_gyro_samples = mis_mat*scale_mat*(gyro_samples(:,i)-bias_vec);
        cur_calib_gyro_samples = cur_calib_gyro_samples * sensitivity;
        calib_gyro_samples = [calib_gyro_samples,cur_calib_gyro_samples];
    end
    % RK4积分
    q = [1,0,0,0];
    for i=1:length(calib_gyro_samples)-1
        omega0 = calib_gyro_samples(1:3,i);
        omega1 = calib_gyro_samples(1:3,i+1);
        q = quat_Integration_StepRK4(q, omega0, omega1, dt);
    end
    q = quaternConj(q);
    rot_mat = quatern2rotMat(q);
    diff = rot_mat * g_versor_pos0 - g_versor_pos1;
    cost_function((i-1)*3+1) = diff(1);
    cost_function((i-1)*3+2) = diff(2);
    cost_function((i-1)*3+3) = diff(3);
end

end