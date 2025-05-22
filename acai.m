function doc = acai(Bf, fmax, fmin, G)
    %ACAI 计算多旋翼基于剩余控制能力的可控度
    %   doc = acai(Bf, fmax, G)
    %       Bf n*m控制效率矩阵
    %       fmax 标量或m维向量表示旋翼产生拉力的最大值
    %       fmin 标量或m维向量表示旋翼产生拉力的最小值
    %       G n维向量表示常值干扰

    % n虚拟控制量个数，m旋翼数量
    [n, m] = size(Bf);

    % 放弃偏航
    if length(G) ~= n
        error('Sizes does not match as G = %s and the prefered length is [%d,1]', mat2str(G), n);
    end

    % 下标矩阵
    S1 = nchoosek(1:m, n - 1);
    % 组合数量
    sm = size(S1, 1);

    if isequal(size(fmax), [1 1])
        fmax = ones(m, 1) * fmax;
    end

    if isequal(size(fmin), [1 1])
        fmin = ones(m, 1) * fmin;
    end

    % 移动区间至边界点
    G = G - Bf * fmin;
    fmax = fmax -fmin;

    % 空间U_f的中心
    fc = fmax / 2;
    % 空间\Omega的中心
    Fc = Bf * fc;

    % 到各组边界的最小值
    dmin = zeros(1, sm);

    for j = 1:sm
        % 选择第j种选择
        choose = S1(j, :);

        % 矩阵B_{1,j}
        B_1j = Bf(:, S1(j, :));
        % 矩阵B_{2,j}
        B_2j = Bf;
        B_2j(:, choose) = [];
        % 对应下标的最大值
        fmax_2 = fmax / 2;
        fmax_2(choose) = [];

        % 法向量
        xi = null(B_1j');
        xi = xi(:, 1);
        e = xi' * B_2j;

        dmin(j) = abs(e) * fmax_2 - abs(xi' * (Fc - G));
    end

    if min(dmin) >= 0
        doc = min(dmin);
    else
        doc = -min(abs(dmin));
    end

    if doc < 1e-10 && doc >- 1e-10
        doc = 0;
    end

end
