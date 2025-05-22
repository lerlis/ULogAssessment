% 模型参数
% 质量和转动惯量
ma = 1.4;
g = 9.8;
Jx = 0.0211;
Jy = 0.0219;
Jz = 0.0366;
J = diag([-ma, Jx, Jy, Jz]);
% 根据油门计算拉力
Cb = 1148/12.6;
Ub = 12.6;
omega_b = -141.4;
ct = 1.105e-05;
f = ct * (Cb * Ub .* sigma_re + omega_b).^2;
% 单个旋翼最大拉力
fmax = ct * (Cb * Ub * 1 + omega_b).^2;

% 动力分配矩阵
d = 0.225;
cm_ct = 0.0161;
% 注意电机编号顺序及转向
Bf = generateBf(4, d, cm_ct, 'initAngle', [45, 225, 315, 135]', 'drct', [1, 1, -1, -1]');
% 最大值
acai_max = acai(Bf, fmax, 0, [ma * g; 0; 0; 0]);
