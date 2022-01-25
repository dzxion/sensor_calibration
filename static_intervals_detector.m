function intervals = static_intervals_detector(data, threshold, h)
% input:
% threshold - 阈值越小，检测越严格
% h - 滑动窗口大小
% min_size - 最小区间
% mean_enable - 用均值算
%
% output:
% intervals - 静态区间结构体数组

intervals = [];% 静态区间
look_for_start = true;
%% var-based
for i=(h+1):length(data)-h
    variance_acc = [var(data(1,i-h:i+h)),var(data(2,i-h:i+h)),var(data(3,i-h:i+h))];
    norm_variance_acc = norm(variance_acc);
    if look_for_start
        if norm_variance_acc < threshold
            current_interval.start_idx = i;
            look_for_start = false;
        end
    else
        if norm_variance_acc >= threshold
            current_interval.end_idx = i-1;
            look_for_start = true;
            intervals = [intervals,current_interval];
        end
    end
end

if look_for_start == false
    current_interval.end_idx = length(data)-h-1;
    intervals = [intervals,current_interval];
end

% n_valid_intervals = 0
% for i=1:length(intervals)
%     if (intervals(i).end_idx - intervals(i).start_idx) >= num
%         n_valid_intervals = n_valid_intervals + 1;
%     end
% end

end