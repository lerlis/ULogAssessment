% 加载数据
load_manipulate_data;
% 导入模型
initialize_model; 

% 状态方程矩阵
n_p = 4;
B = [eye(4) * ts^2/2; eye(4) * ts; zeros(n_p, 4)] / J;
C = [eye(8), zeros(8, n_p)];

%初始化
P = eye(12) * 0.1;
X = zeros(12, 1);
I = eye(size(P));

p_var = 0.005;
Q = eye(12) * p_var^2;

m_var = 1;
R = eye(8) * m_var^2;

% 最终结果
len = length(target_timevec);
eta = zeros(n_p, len);
doc = zeros(1, len);

for i = 1:1:len
    Gamma_f = diag(f(:, i));
    M = -J \ Bf * Gamma_f;
    A = [eye(4) eye(4) * ts M * ts^2/2;
        zeros(4) eye(4) M * ts;
        zeros(n_p, 8), eye(n_p)];

    X_pre = A * X + B * (Bf * f(:, i) - [ma * g; 0; 0; 0]);
    P_pre = A * P * A' + Q;
    K = P_pre * C' / (C * P_pre * C' + R);

    Z_k = [z_re(i); euler_re(:, i); vz_re(i); omega_re(:, i)];
    X = X_pre + K * (Z_k - C * X_pre);
    P = (I - K * C) * P_pre;
    % 保存结果
    eta(:, i) = X(9:12);
    doc(i) = acai(Bf, fmax .* (ones(4, 1) - eta(:, i)), 0, [ma * g, 0, 0, 0]') / acai_max;
end

figure('Name', 'Experiment Results: Loss Factors', 'Position', [0 0 1200 700]);
subplot(2, 2, 1);
plot(target_timevec / onesec, eta(1, :), 'k', 'linewidth', 1.0);
xlabel('time(s)', 'FontName', 'Times New Roman', 'FontSize', 15)
ylabel('loss facotr of the propulsor #1 - \eta_1', 'FontName', 'Times New Roman', 'FontSize', 15)
grid on;hold on;
inject_para = inject_para_line(47,258,106,30,0.6,0.1);
plot(inject_para(:,1), inject_para(:,2), 'r', 'linewidth', 2.0, 'linestyle', ':');
legend('The estimated factor','Real loss factor');
ylim([-0.2 1])

subplot(2, 2, 2);
plot(target_timevec / onesec, eta(2, :), 'k', 'linewidth', 1.0);
xlabel('time(s)', 'FontName', 'Times New Roman', 'FontSize', 15)
ylabel('loss facotr of the propulsor #2 - \eta_2', 'FontName', 'Times New Roman', 'FontSize', 15)
grid on; hold on;
inject_para = inject_para_line(47,258,0,0,0,0.1);
plot(inject_para(:,1), inject_para(:,2), 'r', 'linewidth', 2.0, 'linestyle', ':');
legend('The estimated factor','Real loss factor');
ylim([-0.2 1])
subplot(2, 2, 3);
plot(target_timevec / onesec, eta(3, :), 'k', 'linewidth', 1.0);
xlabel('time(s)', 'FontName', 'Times New Roman', 'FontSize', 15)
ylabel('loss facotr of the propulsor #3 - \eta_3', 'FontName', 'Times New Roman', 'FontSize', 15)
grid on;hold on;
inject_para = inject_para_line(47,258,126,60,0.5,0.1);
plot(inject_para(:,1), inject_para(:,2), 'r', 'linewidth', 2.0, 'linestyle', ':');
legend('The estimated factor','Real loss factor');
ylim([-0.2 1])
subplot(2, 2, 4);
plot(target_timevec / onesec, eta(4, :), 'k', 'linewidth', 1.0);
xlabel('time(s)', 'FontName', 'Times New Roman', 'FontSize', 15)
ylabel('loss facotr of the propulsor #4 - \eta_4', 'FontName', 'Times New Roman', 'FontSize', 15)
grid on; hold on;
inject_para = inject_para_line(47,258,116,10,0.8,0.1);
plot(inject_para(:,1), inject_para(:,2), 'r', 'linewidth', 2.0, 'linestyle', ':');
legend('The estimated factor','Real loss factor');
ylim([-0.2 1])

figure('Name', 'Change of Degree of Contorllability');
plot(target_timevec / onesec, doc, 'k')
xlabel('time(s)', 'FontName', 'Times New Roman', 'FontSize', 15)
ylabel('Degree of Controllability - \rho', 'FontName', 'Times New Roman', 'FontSize', 15)
grid on
ylim([-0.1, 1.1])
