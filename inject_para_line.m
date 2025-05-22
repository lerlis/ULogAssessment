function [line_array] = inject_para_line(start_time, end_time, inject_time, last_time, para_num, sample)
%INJECT_PARA_LINE 此处显示有关此函数的摘要
%   start_time 为图像绘制的起点时刻（s）
%   end_time 为图像绘制的终止时刻（s）
%   inject_time 为故障参数的注入时刻
%   last_time  为故障的持续时间
%   para_num 为注入的故障参数的大小
%   sample 为采样率（指单位步长）
t = start_time : sample : end_time;
len = length(t);
array = zeros(1, len);
for i = 0 : 1 : len
    if ((start_time + sample * i) >= inject_time)&&((start_time + sample * i) <= (inject_time+last_time))
        array(i) = para_num;
end
line_array = [t', array'];
end

