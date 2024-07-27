function [T]= manualCholesky(A)
% 输入：A为一个n x n的实对称正定矩阵
% 输出：L为A的Cholesky分解得到的下三角矩阵
%Input:A is an n x n real symmetric positive definite matrix
%Output:T is the lower triangular matrix obtained by the Cholesky decomposition of A

n = size(A, 1); % 获取矩阵的维度
L = sym(zeros(n)); % 初始化L为n x n的零矩阵

% Cholesky分解的实现
for i = 1:n
    for k = 1:i
        sum = 0;
        for j = 1:(k-1)
            sum = sum + L(i,j) * L(k,j);
        end
        
        if i == k % 对角线元素
            L(i,i) = sqrt(A(i,i) - sum); % 计算对角线元素
        else
            L(i,k) = (A(i,k) - sum) / L(k,k); % 计算非对角线元素
        end
    end
end
T=L;
end