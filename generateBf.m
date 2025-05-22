function Bf = generateBf(np, d, ku, varargin)
    %generateBf 根据参数生成n*np控制效率矩阵
    %   Bf = generateBf(np, d, ku)
    %       np 旋翼数量
    %       d 标量或np维向量，旋翼中心与机体中心距离
    %       ku 标量或np维向量，旋翼转矩系数与了拉力系数的比值
    %   Bf = generateBf(____, Name, Value)
    %       可选参数为
    %       initAngle 第一个旋翼与x轴正向的角度或各个电机的角度向量，角度制，默认为0
    %       drct np维向量表示转动方向，1为逆时针，-1为顺时针，默认PN交替且1号桨为顺时针
    %       eta 标量或np维向量，效率系数，默认为1
    %       giveupYaw 放弃偏航控制，默认为false
    %       giveupHeight 放弃高度控制，默认为false

    % 默认初始角度
    initAngle = 0;
    % 默认转动方向为PN交替且1号桨为逆时针
    drct = (-1).^(1:np);
    % 默认效率为1
    eta = ones(np, 1);
    % 放弃偏航
    giveupYaw = false;
    % 放弃高度
    giveupHeight = false;

    k = length(varargin);

    for i = 1:2:k

        switch varargin{1, i}
            case 'initAngle'
                initAngle = varargin{1, i + 1};
            case 'drct'
                drct = varargin{1, i + 1};
            case 'eta'
                eta = varargin{1, i + 1};
            case 'giveupYaw'
                giveupYaw = varargin{1, i + 1};
            case 'giveupHeight'
                giveupHeight = varargin{1, i + 1};
            otherwise
                warning('参数%s未定义！\n', varargin{1, i});
        end

    end

    % 距离为标量则扩充到向量
    if isequal(size(d), [1 1])
        d = ones(np, 1) * d;
    end

    if isequal(size(ku), [1 1])
        ku = ones(np, 1) * ku;
    end

    if isequal(size(eta), [1 1])
        eta = diag(ones(np, 1) * eta);
    else
        eta = diag(eta);
    end

    % 各电机与x轴正向夹角
    if isequal(size(initAngle), [1 1])
        % 仅定义1号电机其余电机顺时针分布
        phi = (0:np - 1) * 360 / np + initAngle;
    elseif isequal(size(initAngle), [np 1])
        phi = initAngle;
    else
        error('Size of "initalAngle" is incorrect!');
    end

    % 控制效率矩阵
    Bf = zeros(4, np);

    for i = 1:np
        % f
        Bf(1, i) = 1;
        % \tau_x
        Bf(2, i) = -d(i) * sind(phi(i));
        % \tau_y
        Bf(3, i) = d(i) * cosd(phi(i));
        % \tau_z
        Bf(4, i) = drct(i) * ku(i);
    end

    if giveupYaw
        Bf(4, :) = [];
    end

    if giveupHeight
        Bf(1, :) = [];
    end

    Bf = Bf * eta;
end
