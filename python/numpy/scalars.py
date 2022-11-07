import numpy as np
# refer to https://www.freecodecamp.org/news/how-to-fix-typeerror-only-size-1-arrays-can-be-converted-to-python-scalars/

y = np.array([2, 4, 6, 8])

vector = np.vectorize(np.int_)
x = vector(y)
print(x)

x = np.array(list(map(np.int_, y)))
print(x)

x = y.astype(int)
print(x)