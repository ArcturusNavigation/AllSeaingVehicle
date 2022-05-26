import numpy as np
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA


rng = np.random.RandomState(0)
n_samples = 20
cov = [[10, 10], [10, 10]]

X = rng.multivariate_normal(mean=[30, -20], cov=cov, size=n_samples)

def get_dir_vector(data):
    pca = PCA(n_components=2)
    pca.fit(data)
    return pca.components_[0]

def sort_buoys_by_dir(buoys):
    data = np.array([[b[0], b[1]] for b in buoys])
    # subtract mean to center the data
    mean = data.mean(axis=0)
    centered_data = data - mean

    dir = get_dir_vector(centered_data)

    # compute array of dot products
    proj = np.einsum('ij,ij->i', centered_data, np.repeat(dir[np.newaxis, :], centered_data.shape[0], axis=0))

    # testing code to instead return the projections of the data onto the line sorted by dot products
    # proj = proj[proj.argsort()]
    # return proj[:, np.newaxis] / np.dot(dir, dir) * np.repeat(dir[np.newaxis, :], centered_data.shape[0], axis=0) + mean

    # sort the data by the dot products
    sorted_data = centered_data[proj.argsort()]

    # add back in the mean
    return sorted_data + mean

print('original data')
print(X)

ax1 = plt.subplot(121)
ax1.scatter(X[:, 0], X[:, 1], alpha=0.3)
for i, txt in enumerate(range(n_samples)):
    ax1.annotate(txt, (X[i, 0], X[i, 1]))
ax1.set_aspect('equal', 'box')
# ax1.set(xlim=(25, 31), ylim=(-24, -18))

X = sort_buoys_by_dir(X)
print('sorted data')
print(X)

ax2 = plt.subplot(122)
ax2.scatter(X[:, 0], X[:, 1], alpha=0.3)
for i, txt in enumerate(range(n_samples)):
    ax2.annotate(txt, (X[i, 0], X[i, 1]))
ax2.set_aspect('equal', 'box')
# ax2.set(xlim=(25, 31), ylim=(-24, -18))

plt.show()