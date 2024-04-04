import numpy as np
from scipy.spatial import KDTree
from sklearn.decomposition import PCA

#a base algorithm to find k nearest points w.r.t. a point of interest
def find_k_neighborhood(k, point_of_interest, points, tree=None, return_index = False):

  #Construct a KDTree from the point cloud
  if tree is None:
    tree = KDTree(points)

  #Compute the k nearest neighbors of the point of interest
  distances, indices = tree.query(point_of_interest, k=k+1)

  #return a list of index or an array of points based on the flag "return_index"
  if return_index:
    return indices
  else:
    return points[indices[1:], :]

def find_k_neighborhood_homogeneous(k, point_of_interest, points, tree=None, r=0.5, return_index=False):

  #Consider more than k points to find the nearest
  n_considered = k*10

  # Construct a KDTree from the point cloud
  if tree is None:
    tree = KDTree(points)

  # Compute the nearest neighbors of the point of interest
  _, indices = tree.query(point_of_interest, n_considered)

  #Exclude the first point which refer to the point of interest
  neighbor_indices = indices[1:]

  #compute the direction of point w.r.t. the point of interest and their mean
  directions = points[neighbor_indices] - point_of_interest
  mean_direction = np.mean(directions, axis=0)

  #select the k index with a more similar direction to the mean direction
  cosine_similarities = np.dot(directions, mean_direction) / (np.linalg.norm(directions, axis=1) * np.linalg.norm(mean_direction))
  cosine_threshold = 0.8
  selected_indices = neighbor_indices[cosine_similarities >= cosine_threshold]
  selected_indices = selected_indices[:k]

  #select the points of the corresponding indices
  neighboors = points[selected_indices]

  #return a list of index or an array of points based on the flag "return_index"
  if len(neighboors)==k:
    if return_index:
      return selected_indices
    else:
      return neighboors
  
  #in the case there are not points with a similarity with the setting threshold
  #take only the k neighboors basedon the k nearest points base algorithm
  else:
    return find_k_neighborhood(k, point_of_interest, points, tree, return_index)

def compute_normal_with_PCA(query_point, neighboors):

  #center the points w.r.t. the query point
  centred_points = neighboors - query_point

  #Fit a PCA on the points to find a normal
  pca = PCA(n_components=3)
  pca.fit(centred_points)
  normal_vector = pca.components_[2, :]
  
  #return the normal vector of thye query point
  return normal_vector/ np.linalg.norm(normal_vector)

def compute_variance(point_of_interest, nearest):

  #concatenate the points of interest with its nearest points 
  points = np.vstack((point_of_interest, nearest))

  #return the variance of a set of points
  return np.var(points, ddof=1)
