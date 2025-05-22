% 加载数据
load_manipulate_data;
% 导入模型
initialize_model;

% 连续状态空间矩阵
A = [zeros(4) eye(4); zeros(4, 8)];
B = [zeros(4); eye(4) / J];
% 离散状态空间矩阵
Ak = eye(8) + ts * A;
Bk = (eye(8) * ts + A * ts^2) * B;
% 干扰系统
Phi = [Ak -Bk; zeros(4, 8) eye(4)];
Psi = [Bk; zeros(4)];
H = [eye(8) zeros(8, 4)];
% noise characteristic
Q = diag([ones(1, 8) * 0.01^2 0.1^2 ones(1, 3) * 0.001^2]);
R = eye(8) * 1^2;
% 初值
X = zeros(12, 1);
X(3) = ma * g;
P = eye(12) * 0.1;

% 最终结果
len = length(target_timevec);
doc = zeros(1, len);

distb = zeros(4, len);

for i = 1:1:len
    X_pre = Phi * X + Psi * Bf * f(:, i);
    P_pre = Phi * P * Phi' + Q;
    K = P_pre * H' / (H * P_pre * H' + R);

    Z_k = [z_re(i); euler_re(:, i); vz_re(i); dot_euler_re(:, i)];
    X = X_pre + K * (Z_k - H * X_pre);
    P = (eye(size(P)) - K * H) * P_pre;
    % 保存结果
    distb(:, i) = X(9:12);
    doc(i) = acai(Bf, fmax, 0, X(9:12)) / acai_max;
end

figure('Name', 'Change of Degree of Contorllability: by Esitimating the Disturbance');
plot(target_timevec / onesec, doc, 'k')
xlabel('time(s)', 'FontName', 'Times New Roman', 'FontSize', 15)
ylabel('Degree of Controllability - \rho', 'FontName', 'Times New Roman', 'FontSize', 15)
grid on;
ylim([-0.2, 1.1])
