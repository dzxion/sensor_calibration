function [extracted_samples,extracted_intervals] = extract_intervals_samples(data, intervals, min_size, mean_enable)
% input:
% min_size - 最小区间
% mean_enable - 用均值算
%
% output:
% extracted_intervals - 静态区间结构体数组
% extracted_samples - 静态样本点

extracted_intervals = [];extracted_samples = [];
for i=1:length(intervals)
    interval_size = intervals(i).end_idx - intervals(i).start_idx + 1;
    if interval_size >= min_size
        extracted_intervals = [extracted_intervals,intervals(i)];
        if mean_enable
            extracted_samples = [extracted_samples,mean(data(1:3,intervals(i).start_idx:intervals(i).end_idx),2)];
        else
            
        end
    end
end

end