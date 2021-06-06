training data without labels
find the data structure.
clustering:
	K-means algorithm:
		Input:
			K (number of clusters)
			Training set
		Executre:
			Randomly initialize K cluster centroids
			Repeat {
				for i = 1 to m
					c(i) := index( from 1 to K) of cluster centroid closest o x(i)
					% c(i) = min_k(||x(i) - u(k)||^2)
				for k = 1 to K
					u(k) := average(means) of points assigned to cluster k
					u(k) = means(sum(i) of c(i) == k))
			}
		Optimization objective:
			J(c(i), u(k)) = min(sum(||x(i) - u)||^2)	
		Initialization is important and the number of K is important to find Elbow
	Notes:
		https://d3c33hcgiwev3.cloudfront.net/_0104e918fbf0762326a187fdcf615f9d_Lecture13.pdf?Expires=1623110400&Signature=UAWQgZfkQ605xkDZ7Ud8qsLzfj25tN6KZYbWRt95gu9avOo4m8n2is1onYQXXAE35unh9eHdgSa2pARDoWHAdB0horW4GB2GnheGz0Cn28Jtao8a4smBPjO~E2FuEuopDrz8zpuTTghB8SHBePvbluLwW8DN5hn4PEmrQab~l0o_&Key-Pair-Id=APKAJLTNE6QMUY6HBC5A
Motivation:
	Data compression or dimension desreasing

Principal Component Analysis:
	https://www.coursera.org/learn/machine-learning/lecture/ZYIPa/principal-component-analysis-algorithm
	https://www.coursera.org/learn/machine-learning/lecture/X8JoQ/reconstruction-from-compressed-representation
	Notes:
		https://d3c33hcgiwev3.cloudfront.net/_a5935af6cf0625a072cb6e2962e4a47f_Lecture14.pdf?Expires=1623110400&Signature=bineIsI6GrMzuGYnKdWxfGJ3j0o-T2ksIEBPOOx952OAVKuVoIn2TxT1NGlDTdZD2Nkwtrl1xhEOEIIHRlaLvt9JikSB6SNX1mfe0ex17~R1nKkmRpO~c6-XMp4rhXgoJ479USZVBuDLIfDl9uJJc0frHtyYzqBrufVeS0pfquE_&Key-Pair-Id=APKAJLTNE6QMUY6HBC5A
