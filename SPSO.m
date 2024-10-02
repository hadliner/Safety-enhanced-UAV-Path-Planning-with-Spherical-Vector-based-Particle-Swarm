function [BestPosition, BestCost] = SPSO(model)

CostFunction = @(x) MyCost(x,model); % 成本函数

nVar = model.n; % 决策变量数 = PSO中的搜索维度 = 路径节点数
VarSize = [1 nVar]; % 决策变量矩阵的大小

% 粒子（变量）的下界和上界
VarMax = struct('x', model.xmax, 'y', model.ymax, 'z', model.zmax, 'r', 2*norm(model.start-model.end)/nVar);
VarMin = struct('x', model.xmin, 'y', model.ymin, 'z', model.zmin, 'r', 0);

% 仰角
AngleRange = pi/4;  % 限制仰角范围以获得更好的解
VarMin.psi = -AngleRange;
VarMax.psi = AngleRange;

% 方位角
% 确定连接起始点和终止点的向量的角度
dirVector = model.end - model.start;
phi0 = atan2(dirVector(2), dirVector(1));
VarMin.phi = phi0 - AngleRange;
VarMax.phi = phi0 + AngleRange;           

% 速度的下界和上界
alpha = 0.5;
VelMax.r = alpha * (VarMax.r - VarMin.r);
VelMin.r = -VelMax.r;
VelMax.psi = alpha * (VarMax.psi - VarMin.psi);
VelMin.psi = -VelMax.psi;                    
VelMax.phi = alpha * (VarMax.phi - VarMin.phi);
VelMin.phi = -VelMax.phi;                     

% PSO参数
MaxIt = 200; % Maximum Number of Iterations
nPop = 500; % Population Size (Swarm Size)

w = 1;  % 惯性权重
wdamp = 0.98; % 惯性权重的衰减系数
c1 = 1.5; % 个体学习系数
c2 = 1.5; % 全局学习系数

% 创建空粒子结构体
empty_particle.Position = [];
empty_particle.Velocity = [];
empty_particle.Cost = [];
empty_particle.Best.Position = [];
empty_particle.Best.Cost = [];

% 初始化全局最佳解
GlobalBest.Cost = inf; % 最小化问题

% 创建一个空的粒子矩阵，每个粒子是一个解（搜索路径）
particle = repmat(empty_particle, nPop, 1);

% 初始化循环
isInit = false;
while (~isInit)
    disp("Initialising...");
    for i = 1:nPop
        % 初始化位置
        particle(i).Position = CreateRandomSolution(VarSize, VarMin, VarMax);

        % 初始化速度
        particle(i).Velocity.r = zeros(VarSize);
        particle(i).Velocity.psi = zeros(VarSize);
        particle(i).Velocity.phi = zeros(VarSize);

        % 评价
        particle(i).Cost = CostFunction(SphericalToCart(particle(i).Position, model));

        % 更新个体最佳解
        particle(i).Best.Position = particle(i).Position;
        particle(i).Best.Cost = particle(i).Cost;

         % 更新全局最佳解
        if particle(i).Best.Cost < GlobalBest.Cost
            GlobalBest = particle(i).Best;
            isInit = true;
        end
    end
end

% 记录每次迭代的最佳解
BestCost = zeros(MaxIt, 1);

% PSO主循环
for it = 1:MaxIt
    % 更新历史最佳解
    BestCost(it) = GlobalBest.Cost;

    for i = 1:nPop          
        % r部分
        % 更新速度
        particle(i).Velocity.r = w * particle(i).Velocity.r ...
            + c1 * rand(VarSize) .* (particle(i).Best.Position.r - particle(i).Position.r) ...
            + c2 * rand(VarSize) .* (GlobalBest.Position.r - particle(i).Position.r);

        % 更新速度限制
        particle(i).Velocity.r = max(particle(i).Velocity.r, VelMin.r);
        particle(i).Velocity.r = min(particle(i).Velocity.r, VelMax.r);

        % 更新位置
        particle(i).Position.r = particle(i).Position.r + particle(i).Velocity.r;

        % 速度镜像
        OutOfTheRange = (particle(i).Position.r < VarMin.r | particle(i).Position.r > VarMax.r);
        particle(i).Velocity.r(OutOfTheRange) = -particle(i).Velocity.r(OutOfTheRange);

        % 更新位置限制
        particle(i).Position.r = max(particle(i).Position.r, VarMin.r);
        particle(i).Position.r = min(particle(i).Position.r, VarMax.r);

        % psi部分
        % 更新速度
        particle(i).Velocity.psi = w * particle(i).Velocity.psi ...
            + c1 * rand(VarSize) .* (particle(i).Best.Position.psi - particle(i).Position.psi) ...
            + c2 * rand(VarSize) .* (GlobalBest.Position.psi - particle(i).Position.psi);

        % 更新速度限制
        particle(i).Velocity.psi = max(particle(i).Velocity.psi, VelMin.psi);
        particle(i).Velocity.psi = min(particle(i).Velocity.psi, VelMax.psi);

        % 更新位置
        particle(i).Position.psi = particle(i).Position.psi + particle(i).Velocity.psi;

        % 速度镜像
        OutOfTheRange = (particle(i).Position.psi < VarMin.psi | particle(i).Position.psi > VarMax.psi);
        particle(i).Velocity.psi(OutOfTheRange) = -particle(i).Velocity.psi(OutOfTheRange);

        % 更新位置限制
        particle(i).Position.psi = max(particle(i).Position.psi, VarMin.psi);
        particle(i).Position.psi = min(particle(i).Position.psi, VarMax.psi);

        % phi部分
         % 更新速度
        particle(i).Velocity.phi = w * particle(i).Velocity.phi ...
            + c1 * rand(VarSize) .* (particle(i).Best.Position.phi - particle(i).Position.phi) ...
            + c2 * rand(VarSize) .* (GlobalBest.Position.phi - particle(i).Position.phi);

        % 更新速度限制
        particle(i).Velocity.phi = max(particle(i).Velocity.phi, VelMin.phi);
        particle(i).Velocity.phi = min(particle(i).Velocity.phi, VelMax.phi);

        % 更新位置
        particle(i).Position.phi = particle(i).Position.phi + particle(i).Velocity.phi;

        % 速度镜像
        OutOfTheRange = (particle(i).Position.phi < VarMin.phi | particle(i).Position.phi > VarMax.phi);
        particle(i).Velocity.phi(OutOfTheRange) = -particle(i).Velocity.phi(OutOfTheRange);

        % 更新位置限制
        particle(i).Position.phi = max(particle(i).Position.phi, VarMin.phi);
        particle(i).Position.phi = min(particle(i).Position.phi, VarMax.phi);

        % 评价
        particle(i).Cost = CostFunction(SphericalToCart(particle(i).Position, model));

        % 更新个体最佳解
        if particle(i).Cost < particle(i).Best.Cost
            particle(i).Best.Position = particle(i).Position;
            particle(i).Best.Cost = particle(i).Cost;

            % 更新全局最佳解
            if particle(i).Best.Cost < GlobalBest.Cost
                GlobalBest = particle(i).Best;
            end
        end
    end

     % 更新惯性权重
    w = w * wdamp;

    % 显示迭代信息
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
end

% 返回最佳解和最佳成本
BestPosition = SphericalToCart(GlobalBest.Position,model);
BestCost = BestCost(1:it);

end