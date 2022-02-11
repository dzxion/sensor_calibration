close all
clear all
clc

ToDeg = 180/pi;ToRad = pi/180;
GravityAcc = 980.665;
IMU_Sensors_Count = 1;
freq_sdlog = 250;
dt = 1/freq_sdlog;
start = 3;

%% 导入数据
path = 'REC/校准3 250hz.REC';
fip=fopen(path,'rb');
if fip==-1
    return
end
k=0;
imu_cnt=0;
gps_cnt=0;
while ~feof(fip) 
    k=k+1;
    head=fread(fip,[1,1],'uint8');
    if size(head,1)==0
        break
    end
    type(k)=fread(fip,[1,1],'uint8');
    leng=fread(fip,[1,1],'uint8');
    check=fread(fip,[1,1],'uint8');
    Time(k)=fread(fip,[1,1],'uint32');
    TTT=Time(k);
    if type(k)==0%imu
        imu_cnt=imu_cnt+1;
        imu_sys_time(imu_cnt)=Time(k);
        
        gyro1_raw_x(imu_cnt)=(fread(fip,[1,1],'float')-32768);
        gyro1_raw_y(imu_cnt)= -(fread(fip,[1,1],'float')-32768);
        gyro1_raw_z(imu_cnt)=(fread(fip,[1,1],'float')-32768);
        
        acc1_raw_x(imu_cnt)=(fread(fip,[1,1],'float')-32768);
        acc1_raw_y(imu_cnt)=(fread(fip,[1,1],'float')-32768);
        acc1_raw_z(imu_cnt)=(fread(fip,[1,1],'float')-32768);
                
%         gyro2_x_raw(imu_cnt)=fread(fip,[1,1],'float');
%         gyro2_y_raw(imu_cnt)=fread(fip,[1,1],'float');
%         gyro2_z_raw(imu_cnt)=fread(fip,[1,1],'float');
%         
%         acc2_x_raw(imu_cnt)=fread(fip,[1,1],'float');
%         acc2_y_raw(imu_cnt)=fread(fip,[1,1],'float');
%         acc2_z_raw(imu_cnt)=fread(fip,[1,1],'float');
%         
%         gyro3_x_raw(imu_cnt)=fread(fip,[1,1],'float');
%         gyro3_y_raw(imu_cnt)=fread(fip,[1,1],'float');
%         gyro3_z_raw(imu_cnt)=fread(fip,[1,1],'float');
%         
%         acc3_x_raw(imu_cnt)=fread(fip,[1,1],'float');
%         acc3_y_raw(imu_cnt)=fread(fip,[1,1],'float');
%         acc3_z_raw(imu_cnt)=fread(fip,[1,1],'float');
%         
%         gyro4_x_raw(imu_cnt)=fread(fip,[1,1],'float');
%         gyro4_y_raw(imu_cnt)=fread(fip,[1,1],'float');
%         gyro4_z_raw(imu_cnt)=fread(fip,[1,1],'float');
%         
%         acc4_x_raw(imu_cnt)=fread(fip,[1,1],'float');
%         acc4_y_raw(imu_cnt)=fread(fip,[1,1],'float');
%         acc4_z_raw(imu_cnt)=fread(fip,[1,1],'float');
    end
end
fclose(fip);

if start == 0
    error('calib disable');
end

gyro1_x = ((gyro1_raw_x) / 16.384);
gyro1_y = ((gyro1_raw_y) / 16.384);
gyro1_z = ((gyro1_raw_z) / 16.384);
acc1_x = ((acc1_raw_x) / 1365);
acc1_y = ((acc1_raw_y) / 1365);
acc1_z = ((acc1_raw_z) / 1365);

% gyro2_x = (gyro2_x_raw - 32768) / 16.384;
% gyro2_y = (gyro2_y_raw - 32768) / 16.384;
% gyro2_z = (gyro2_z_raw - 32768) / 16.384;
% acc2_x = (acc2_x_raw - 32768) / 1365;
% acc2_y = (acc2_y_raw - 32768) / 1365;
% acc2_z = (acc2_z_raw - 32768) / 1365;
% 
% gyro3_x = (gyro3_x_raw - 32768) / 16.384;
% gyro3_y = (gyro3_y_raw - 32768) / 16.384;
% gyro3_z = (gyro3_z_raw - 32768) / 16.384;
% acc3_x = (acc3_x_raw - 32768) / 1365;
% acc3_y = (acc3_y_raw - 32768) / 1365;
% acc3_z = (acc3_z_raw - 32768) / 1365;
% 
% gyro4_x = (gyro4_x_raw - 32768) / 16.384;
% gyro4_y = (gyro4_y_raw - 32768) / 16.384;
% gyro4_z = (gyro4_z_raw - 32768) / 16.384;
% acc4_x = (acc4_x_raw - 32768) / 1365;
% acc4_y = (acc4_y_raw - 32768) / 1365;
% acc4_z = (acc4_z_raw - 32768) / 1365;

%% 滤波
fs=50;fc=5;
[b,a] = butter(4,fc/(fs/2));
gyro1_filter_x = filter(b,a,gyro1_x);
gyro1_filter_y = filter(b,a,gyro1_y);
gyro1_filter_z = filter(b,a,gyro1_z);
acc1_filter_x = filter(b,a,acc1_x);
acc1_filter_y = filter(b,a,acc1_y);
acc1_filter_z = filter(b,a,acc1_z);

acc_raw = [acc1_raw_x;acc1_raw_y;acc1_raw_z];
gyro_raw = [gyro1_raw_x;gyro1_raw_y;gyro1_raw_z];

% norm_raw_acc = zeros(1,length(acc1_raw_x));
% for i=1:length(acc1_raw_x)
%     vec = [acc1_raw_x(i),acc1_raw_y(i),acc1_raw_z(i)];
%     norm_raw_acc(i) = norm(vec);
% end

%% 初始静止阶段
init_interval_duration = 10;
end_idx = init_interval_duration * freq_sdlog;
start_idx = 1 * freq_sdlog;
% 加速度方差
variance_acc_x = var(acc_raw(1,start_idx:end_idx));
variance_acc_y = var(acc_raw(2,start_idx:end_idx));
variance_acc_z = var(acc_raw(3,start_idx:end_idx));
variance_acc = [variance_acc_x,variance_acc_y,variance_acc_z];
norm_variance_acc = norm(variance_acc);
% 陀螺零偏
gyro_bias = mean(gyro_raw(1:3,start_idx:end_idx),2);

if start == 1
    error('calib static stage');
end

%% 加速度计校准
acc_calib_params_9 = [];resnorm_9 = [];
acc_calib_params_12 = [];resnorm_12 = [];
extracted_samples = [];extracted_intervals = [];
error_calib_9 = [];error_calib_12 = [];

min_error = 100;
min_error_k = -1;
min_error_static_intervals = [];
min_error_acc_calib_params = [];
init_acc_calib_9 =[0,0,0,1,1,1,0,0,0];
init_acc_calib_12 =[0,0,0,0,0,0,1,1,1,0,0,0];
win_time_sec = 1;
win_half = (win_time_sec/2)*freq_sdlog;win = win_time_sec * freq_sdlog;mean_enable = true;
fprintf('Calibration Accelerometer:\n');
for i=1:IMU_Sensors_Count
%     for k=2:10
        % 静态检测
        k = 10;
        threshold = k * norm_variance_acc;
        intervals = static_intervals_detector(acc_raw,threshold,win_half);
        [cur_extracted_samples,cur_extracted_intervals] = extract_intervals_samples(acc_raw, intervals, win, mean_enable);
        %     extracted_samples = [extracted_samples;cur_extracted_samples];
        %     extracted_intervals = [extracted_intervals;cur_extracted_intervals];
%         for i=1:length(cur_extracted_samples)
% %             fprintf('sample %d (before calibration)\n',i);
% %             fprintf('acceleration before calibration :');
% %             cur_extracted_samples(:,i)
%             fprintf('error norm : %d\n',abs(norm(cur_extracted_samples(:,i))-1365) );
%         end

        for i=1:length(cur_extracted_samples)
             norm_before(i) = norm(cur_extracted_samples(:,i));
        end
        error_before = max(abs(norm_before - 1365*ones(1,length(norm_before))));
        fprintf('error max norm before : %d\n',error_before );
        
        % lsqnonlin优化
        options=optimset('TolX',1e-6,'TolFun',1e-6,'Algorithm','Levenberg-Marquardt');
        %         [cur_acc_calib_params_9, cur_resnorm_9] = lsqnonlin(@(cur_acc_calib_params_9) cost_acc(cur_acc_calib_params_9,cur_extracted_samples,9), init_acc_calib_9);
        %         acc_calib_params_9 = [acc_calib_params_9;cur_acc_calib_params_9];resnorm_9 = [resnorm_9;cur_resnorm_9];
        %         [cur_acc_calib_params_12, cur_resnorm_12] = lsqnonlin(@(cur_acc_calib_params_12) cost_acc(cur_acc_calib_params_12,cur_extracted_samples,12), init_acc_calib_12);
        %         acc_calib_params_12 = [acc_calib_params_12;cur_acc_calib_params_12];resnorm_12 = [resnorm_12;cur_resnorm_12];
        
%         [cur_acc_calib_params_9,cur_resnorm_9] = lsqnonlin(@(acc_calib_params) cost_acc_9(acc_calib_params,cur_extracted_samples), init_acc_calib_9, [],[],options);
%         acc_calib_params_9 = [acc_calib_params_9;cur_acc_calib_params_9];resnorm_9 = [resnorm_9;cur_resnorm_9];
%         acc_calib_ext_9 = unbiasNormalize_9(cur_extracted_samples, cur_acc_calib_params_9);
%         norm_calib_acc = [];
%         for i=1:length(cur_extracted_samples)
%             norm_calib_acc(i) = norm(acc_calib_ext_9(:,i));
%         end
%         error_calib_9 = [error_calib_9;mean(abs(norm_calib_acc - 1365*ones(1,length(norm_calib_acc))))];
        
        [cur_acc_calib_params_12,cur_resnorm_12] = lsqnonlin(@(acc_calib_params) cost_acc_12(acc_calib_params,cur_extracted_samples), init_acc_calib_12,[],[], options);
%         acc_calib_params_12 = [acc_calib_params_12;cur_acc_calib_params_12];resnorm_12 = [resnorm_12;cur_resnorm_12];
%         acc_calib_ext_12 = unbiasNormalize_12(cur_extracted_samples, cur_acc_calib_params_12);
%         norm_calib_acc = [];
%         for i=1:length(cur_extracted_samples)
%             norm_calib_acc(i) = norm(acc_calib_ext_12(:,i));
%         end
%         error_calib_12 = max(abs(norm_calib_acc - 1365*ones(1,length(norm_calib_acc))));
%         if error_calib_12 < min_error
%             min_error = error_calib_12;
%             min_error_k = k;
%             min_error_static_intervals = cur_extracted_intervals;
%             min_error_acc_calib_params = cur_acc_calib_params_12;
%         end
%     end
end
calib_acc_samples = unbiasNormalize_12(acc_raw,cur_acc_calib_params_12);

%% 画静态检测效果
Time = Time * 1e-3;
plotintervals = zeros(1,length(acc1_raw_x));
k = 1e3;
for i=1:length(cur_extracted_intervals)
    plotintervals(1,cur_extracted_intervals(i).start_idx:cur_extracted_intervals(i).end_idx) = k * ones(1,cur_extracted_intervals(i).end_idx - cur_extracted_intervals(i).start_idx + 1);
end

figure
plot(Time,acc1_raw_x);
grid on;hold on;
plot(Time,acc1_raw_y);
grid on;hold on;
plot(Time,acc1_raw_z);
grid on;hold on;
plot(Time,plotintervals);
xlabel('time');
ylabel('acc raw');
legend('acc raw x','acc raw y','acc raw z');

if start == 2
    error('calib accelerometer finish');
end

%% 加速度校准评估
extracted_samples = [];extracted_intervals = [];
[extracted_samples,extracted_intervals] = extract_intervals_samples(calib_acc_samples, cur_extracted_intervals, win, mean_enable);
% for i=1:length(extracted_samples)
% %             fprintf('sample %d (before calibration)\n',i);
% %             fprintf('acceleration before calibration :');
% %             cur_extracted_samples(:,i)
%     fprintf('error norm : %d\n',abs(norm(extracted_samples(:,i))-1365) );
% end
for i=1:length(extracted_samples)
    norm_after(i) = norm(extracted_samples(:,i));
end
error_after = max(abs(norm_after - 1365*ones(1,length(norm_after))));
fprintf('error max norm after: %d\n',error_after );

%% 陀螺校准
sensitivity = ToRad / 16.384;
gyro_calib = [0,0,0,0,0,0,1,1,1,gyro_bias(1),gyro_bias(2),gyro_bias(3)];
gyro_unbias = unbiasNormalize_12(gyro_raw,gyro_calib);

init_gyro_calib =[0,0,0,0,0,0,1,1,1];
fprintf('Calibration Gyroscope:\n');

options=optimset('TolX',1e-6,'TolFun',1e-6,'Algorithm','Levenberg-Marquardt');
for i=1:IMU_Sensors_Count
    [gyro_calib_params,gyro_calib_resnorm] = lsqnonlin(@(gyro_calib_params) cost_gyro( gyro_calib_params, gyro_unbias, dt, extracted_samples, extracted_intervals, sensitivity), init_gyro_calib, [],[],options);
%     calib_gyro_samples = unbiasNormalize_12(gyro_raw,gyro_calib_params);
    gyro_calib_params = [gyro_calib_params,gyro_bias(1),gyro_bias(2),gyro_bias(3)];
end
fprintf('Calibration finish:\n');

%% 陀螺校准评估
gyro_calib_before = gyro_unbias * sensitivity;
gyro_calib_after = unbiasNormalize_12(gyro_raw,gyro_calib_params) * sensitivity;
error_angle_before = [];
error_angle_after = [];

for i=1:length(extracted_samples)-1
    g_versor_pos0 = extracted_samples(1:3,i);
    g_versor_pos1 = extracted_samples(1:3,i+1);
    g_versor_pos0 = g_versor_pos0 / norm(g_versor_pos0);
    g_versor_pos1 = g_versor_pos1 / norm(g_versor_pos1);
    gyro_interval.start_idx = extracted_intervals(i).end_idx;
    gyro_interval.end_idx = extracted_intervals(i+1).start_idx;
    
    gyro_samples_before = gyro_calib_before(:,gyro_interval.start_idx:gyro_interval.end_idx);
    gyro_samples_after = gyro_calib_after(:,gyro_interval.start_idx:gyro_interval.end_idx);
    % RK4积分
    q_before = [1,0,0,0];
    q_after = [1,0,0,0];
    for i=1:length(gyro_samples_before)-1
        omega0 = gyro_samples_before(1:3,i);
        omega1 = gyro_samples_before(1:3,i+1);
        q_before = quat_Integration_StepRK4(q_before, omega0, omega1, dt);
        
        omega0 = gyro_samples_after(1:3,i);
        omega1 = gyro_samples_after(1:3,i+1);
        q_after = quat_Integration_StepRK4(q_after, omega0, omega1, dt);
    end
    q_before = quaternConj(q_before);
    q_after = quaternConj(q_after);
    rot_mat_before = quatern2rotMat(q_before);
    rot_mat_after = quatern2rotMat(q_after);
%     diff = rot_mat * g_versor_pos0 - g_versor_pos1;
    [angle,axis] = get_included_angle_from_unit_vector(rot_mat_before * g_versor_pos0, g_versor_pos1);
    error_angle_before = [error_angle_before,abs(angle)*ToDeg];
%     fprintf('error angle before: %d \n',abs(angle)*ToDeg);
    
    [angle,axis] = get_included_angle_from_unit_vector(rot_mat_after * g_versor_pos0, g_versor_pos1);
    error_angle_after = [error_angle_after,abs(angle)*ToDeg];
%     fprintf('error angle after: %d \n',abs(angle)*ToDeg);
end

fprintf('error max angle before: %d\n',max(error_angle_before) );
fprintf('error max angle after: %d\n',max(error_angle_after) );

%% 校准结果显示
figure
plot(norm_before*1000/1365);
grid on;hold on;
plot(norm_after*1000/1365);
grid on;hold on;
ylabel('accelration (mg)');
legend('before','after');
