Multi-Class Neural Networks: Softmax bookmark_border
	Softmax extends this idea into a multi-class world. That is, Softmax assigns decimal probabilities to each class in a multi-class problem.
	Those decimal probabilities must add up to 1.0. This additional constraint helps training converge more quickly than it otherwise would.

Softmax Options
	Consider the following variants of Softmax:
		Full Softmax is the Softmax we've been discussing; that is, Softmax calculates a probability for every possible class.
		Candidate sampling means that Softmax calculates a probability for all the positive labels but only for a random sample of negative labels.
			For example, if we are interested in determining whether an input image is a beagle or a bloodhound, we don't have to provide probabilities for every non-doggy example.

		Full Softmax is fairly cheap when the number of classes is small but becomes prohibitively expensive when the number of classes climbs.
			Candidate sampling can improve efficiency in problems having a large number of classes.

	reference to https://developers.google.com/machine-learning/crash-course/multi-class-neural-networks/softmax

def softmax(x):
    """Compute softmax values for each sets of scores in x. x is the vector of cnn results"""
    return np.exp(x) / np.sum(np.exp(x), axis=0) 

	""" reference to https://medium.com/data-science-bootcamp/understand-the-softmax-function-in-minutes-f3a59641e86d"""

reference to :
	https://towardsdatascience.com/softmax-function-simplified-714068bf8156
	https://gist.github.com/hamza121star/bc72856bfd488f5c62274dca58734c51#file-softmax-py

	import numpy as np
	import tensorflow as tf
	
	""" 
	Input --> scores of word embeddings
	Output --> return a single softmax value
	"""
	
	#Numpy version of Softmax function
	def softmax_numpy(scores):
	  return np.exp(scores)/sum(np.exp(scores), axis=0)
	
	#Tensorflow version of Softmax function
	def softmax_tensorflow(scores):
	  return tf.exp(scores)/tf.reducesum(tf.exp(scores), 1)

