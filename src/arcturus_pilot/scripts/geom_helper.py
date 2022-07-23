import numpy as np
from geometry_msgs.msg import Quaternion
from sklearn.decomposition import PCA
import tf

def angle_from_dir(dir):
    return np.arctan2(dir[1], dir[0])

def quaternion_from_angle(angle):
    q = tf.transformations.quaternion_from_euler(0, 0, angle)
    return Quaternion(q[0], q[1], q[2], q[3])

def quaternion_from_dir(dir):
    return quaternion_from_angle(angle_from_dir(dir))

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
