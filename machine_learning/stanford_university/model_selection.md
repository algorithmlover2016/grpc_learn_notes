evaluate a hypothesis with testing set directly:
	https://www.coursera.org/learn/machine-learning/supplement/aFpD3/evaluating-a-hypothesis
	1. take cost function to compute the error with testing set.
	2. classificatoin ~ Misclassification error(0 / 1)

Split Data set into three parts: training set(60%), cross validatoin set(20%), testing set(20%).
	reference to https://www.coursera.org/learn/machine-learning/supplement/XHQqO/model-selection-and-train-validation-test-sets
	1. generate the cost function with different degree of polynomial and then use training set to get theta parameters
	2. use cross validation set to select whih degree will min(J(theta))
	3. use testing set to compute cost function

Bias and Variance:
	High Bias means underfitting
	High Variance means overfitting
	The training error will tend to decrease as we increase the degree d of the polynomial.
	At the same time, the cross validation error will tend to decrease as we increase d up to a point, and then it will increase as d is increased,
		forming a convex curve.

	regularization_with_bias_and_variance
		as λ increases, our fit becomes more rigid. On the other hand, as λ approaches 0, we tend to over overfit the data.
		reference to https://www.coursera.org/learn/machine-learning/supplement/JPJJj/regularization-and-bias-variance
	
	learning curves with training set increasing
		https://www.coursera.org/learn/machine-learning/supplement/79woL/learning-curves
	
	decide what to do:
		https://www.coursera.org/learn/machine-learning/supplement/llc5g/deciding-what-to-do-next-revisited
