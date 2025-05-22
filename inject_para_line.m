function [line_array] = inject_para_line(start_time, end_time, inject_time, last_time, para_num, sample)
%INJECT_PARA_LINE �˴���ʾ�йش˺�����ժҪ
%   start_time Ϊͼ����Ƶ����ʱ�̣�s��
%   end_time Ϊͼ����Ƶ���ֹʱ�̣�s��
%   inject_time Ϊ���ϲ�����ע��ʱ��
%   last_time  Ϊ���ϵĳ���ʱ��
%   para_num Ϊע��Ĺ��ϲ����Ĵ�С
%   sample Ϊ�����ʣ�ָ��λ������
t = start_time : sample : end_time;
len = length(t);
array = zeros(1, len);
for i = 0 : 1 : len
    if ((start_time + sample * i) >= inject_time)&&((start_time + sample * i) <= (inject_time+last_time))
        array(i) = para_num;
end
line_array = [t', array'];
end

