function [U, S] = pca(X)
%PCA Run principal component analysis on the dataset X
%   [U, S, X] = pca(X) computes eigenvectors of the covariance matrix of X
%   Returns the eigenvectors U, the eigenvalues (on diagonal) in S
%

% Useful values
[m, n] = size(X);

% You need to return the following variables correctly.
U = zeros(n);
S = zeros(n);

% ====================== YOUR CODE HERE ======================
% Instructions: You should first compute the covariance matrix. Then, you
%               should use the "svd" function to compute the eigenvectors
%               and eigenvalues of the covariance matrix. 
%
% Note: When computing the covariance matrix, remember to divide by m (the
%       number of examples).
%
% reference to https://www.cnblogs.com/geeksongs/p/11190295.html and
%              https://www.zybuluo.com/EtoDemerzel/note/958976 and
%              https://zhuanlan.zhihu.com/p/349802953
% X(i,: ) is an input point, and mean(X(i, :)) = 0
[U, S, V] = svd(1 / (m) * X' * X);
% [U, S, V] = svd(1 / (m - 1) * X' * X);


% =========================================================================

end
