import numpy as np
from scipy.spatial import KDTree
from sklearn.decomposition import PCA

def umbrella_method(query_point, points, normal):

  s = 0
  d = 0
  
  #sum the projections of the nearest points on the normal of the query point 
  for pi in points:
    d = (pi-query_point)/np.linalg.norm(pi-query_point)
    s+=np.abs(np.dot(d,normal))

  return s

#this method aim to increase robustness to noise 
def umbrella_method2(query_point, points, normal):

  s = 0
  d = 0

  #cenetr the points w.r.t. the median
  median = np.median(points, axis=0)
  points = points - median

  #sum the projections of the nearest points on the normal of the query point
  for pi in points:
    d = (pi-query_point)/np.linalg.norm(pi-query_point)
    s+=np.abs(np.dot(d,normal))

  return s

#this method aim to increase robustness to noise 
def umbrella_method3(query_point, points, normal):

  s = 0
  d = 0
  filtered_points = np.copy(points)
  num_points = points.shape[0]
  window_size = 3

  #define a window
  window = []
  for i in range(window_size):
    window.append(np.zeros(3))

  #for each point
  for i in range(num_points):

    #update the window
    window[:-1] = window[1:]
    window[-1] = points[i]

    #the new point is the median of the window's values
    filtered_points[i] = np.median(window,axis =0)

  #sum the projections of the nearest points on the normal of the query point
  for pi in filtered_points:
    d = (pi-query_point)/np.linalg.norm(pi-query_point)
    s+=np.abs(np.dot(d,normal))

  return s

def surface_variance_method(query_point, neighboors):

  #centerthe points w.r.t. the query point
  centred_points = neighboors - query_point

  #Fit a PCA on the points
  pca = PCA(n_components=3)
  pca.fit(centred_points)

  #compute the curvature as the ratio between the minimum eigenvalue and the sum of all eigenvalues 
  eigenvalues = pca.explained_variance_
  surface_variance = min(eigenvalues)/sum(eigenvalues)
  
  return surface_variance

#compute the curvature for each point of the point cloud based on the selected method
def point_cloud_curvatures(point_cloud, k=8, method='umbrella'):

  curvatures_pc = []

  if method=='umbrella':
    for p in point_cloud:
      neighboors = find_k_neighborhood_homogeneous(k, p, point_cloud)
      normal_vector = compute_normal_with_PCA(p, neighboors)
      curvature = umbrella_method(p, neighboors, normal_vector)
      curvatures_pc.append(curvature)

  elif method=='umbrella2':
    for p in point_cloud:
      neighboors = find_k_neighborhood_homogeneous(k, p, point_cloud)
      normal_vector = compute_normal_with_PCA(p, neighboors)
      curvature = umbrella_method2(p, neighboors, normal_vector)
      curvatures_pc.append(curvature)

  elif method=='umbrella3':
    for p in point_cloud:
      neighboors = find_k_neighborhood_homogeneous(k, p, point_cloud)
      normal_vector = compute_normal_with_PCA(p, neighboors)
      curvature = umbrella_method3(p, neighboors, normal_vector)
      curvatures_pc.append(curvature)

  elif method=='surface_variance':
    for p in point_cloud:
      neighboors = find_k_neighborhood(k, p, point_cloud)
      curvature = surface_variance_method(p, neighboors)
      curvatures_pc.append(curvature)

  else:
    print("Method not supported")

  return curvatures_pc #list of curvatures
