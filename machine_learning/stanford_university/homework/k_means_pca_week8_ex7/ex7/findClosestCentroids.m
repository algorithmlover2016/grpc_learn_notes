function idx = findClosestCentroids(X, centroids)
%FINDCLOSESTCENTROIDS computes the centroid memberships for every example
%   idx = FINDCLOSESTCENTROIDS (X, centroids) returns the closest centroids
%   in idx for a dataset X where each row is a single example. idx = m x 1 
%   vector of centroid assignments (i.e. each entry in range [1..K])
%

% Set K
K = size(centroids, 1);

% You need to return the following variables correctly.
idx = zeros(size(X,1), 1);

% ====================== YOUR CODE HERE ======================
% Instructions: Go over every example, find its closest centroid, and store
%               the index inside idx at the appropriate location.
%               Concretely, idx(i) should contain the index of the centroid
%               closest to example i. Hence, it should be a value in the 
%               range 1..K
%
% Note: You can use a for-loop over the examples to compute this.
%
% reference to https://wangpei.ink/2019/03/05/Coursera-%E6%9C%BA%E5%99%A8%E5%AD%A6%E4%B9%A0-(%E5%90%B4%E6%81%A9%E8%BE%BE)%E7%BC%96%E7%A8%8B%E4%BD%9C%E4%B8%9A%E7%AC%AC%E5%85%AB%E5%91%A8(ex7)/
distances = zeros(K, 1);
for i = 1 : length(idx)
    ele = X(i, :);
    distances = sum((ele(ones(K, 1), :) - centroids) .^ 2, 2);
    [val, minIdx] = min(distances);
    idx(i) = minIdx;
end


% =============================================================

end

