import numpy as np
from plot_utility import *
from scipy.spatial import KDTree
from sklearn.decomposition import PCA
from neighboors import *

#compute the euclidean distance between two points
def euclidean_distance(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

#Look for the points with similar curvature feature
def find_correlations(source_pc, source_curvatures, target_pc, target_curvatures):

    #lists to collect similar points
    new_source_pc = []
    new_target_pc = []
    
    #KDTree to find neighboors
    target_tree = KDTree(target_pc)

    #Loop to confront each points of the source with its closest of the target
    for p_source,c_source in zip(source_pc, source_curvatures):

      #find neighboors, take its curvature and take the difference of them with point curvature of source 
      near_points_idx = find_k_neighborhood(50, p_source, target_pc, target_tree, return_index=True)
      curv = [target_curvatures[i] for i in near_points_idx]
      diff = [np.abs(c_source-c_target) for c_target in curv]

      #chek if the more similar point falls within the treshold
      if len(diff)>0 and min(diff) <= 0.0001:
        i_target = near_points_idx[np.argmin(diff)]
        p_target = target_pc[i_target]
        new_source_pc.append(p_source)
        new_target_pc.append(p_target)

    #check lenght
    assert len(new_source_pc) == len(new_target_pc)
        
    #if no match are foud
    if len(new_source_pc)==0:
      return None,None

    #return two list of points with euqal lenght
    return new_source_pc, new_target_pc

#Look for the points with similar curvature feature and similar local variance
def find_correlations2(source_pc, source_curvatures, target_pc, target_curvatures):

    #lists to collect similar points
    new_source_pc = []
    new_target_pc = []
    
    #KDTree to find neighboors
    target_tree = KDTree(target_pc)
      
    #Loop to confront each points of the source with its closest of the target
    for p_source,c_source in zip(source_pc, source_curvatures):

      #find neighboors, take its curvature and take the difference of them with point curvature of source 
      near_points_idx = find_k_neighborhood(50, p_source, target_pc, target_tree, return_index=True)
      curv = [target_curvatures[i] for i in near_points_idx]
      diff = [np.abs(c_source-c_target) for c_target in curv]

      #1st chek: if the more similar point falls within the treshold
      if len(diff)>0 and min(diff) <= 0.0001:

        i_target = near_points_idx[np.argmin(diff)]
        p_target = target_pc[i_target]

        #find set of closest points both for source and target
        nearest_source = find_k_neighborhood(1000, p_source, source_pc)
        nearest_target = find_k_neighborhood(1000, p_target, target_pc, target_tree)

        #2nd check: if the two points have a similar variance in the distribution of their neighboors
        if np.abs(compute_variance(p_source,nearest_source)-compute_variance(p_target,nearest_target))<0.0001:
          new_source_pc.append(p_source)
          new_target_pc.append(p_target)

    #check lenght
    assert len(new_source_pc) == len(new_target_pc)

    #if no match are foud
    if len(new_source_pc)==0:
      return None,None

    #return two list of points with euqal lenght
    return new_source_pc, new_target_pc

#compute the matrix to let transformation between source and target point clouds
def compute_transformation_matrix(new_source_pc, new_target_pc):

  #centroid of teh two set of points
  source_centroid = np.mean(new_source_pc, axis=0)
  target_centroid = np.mean(new_target_pc, axis=0)

  #center the points with to respect their centroids
  new_source_pc = new_source_pc - source_centroid
  new_target_pc = new_target_pc - target_centroid

  #compute covariance matrix
  cov_mat = np.dot(new_source_pc.T, new_target_pc)

  #decompose covariance matrix with Singualr Value Decomposition
  U, S, Vt = np.linalg.svd(cov_mat)

  #compute rotation matrix
  R = np.dot(Vt.T,U.T)

  #compute translation matrix
  t = target_centroid - np.dot(R,source_centroid)

  #compute transformation matrix
  T = np.eye(4)
  T[:3,:3] = R
  T[:3,3] = t

  #return a matrix 3x3
  return T

#Apply the transformation matrix to a point cloud
def point_cloud_transformation(transformation_matrix, point_cloud):

  #convert in homogeneous coordinates
  pc_hom = np.hstack((point_cloud, np.ones((point_cloud.shape[0], 1))))

  #apply transformation
  pc_transformed_hom = np.dot(transformation_matrix, pc_hom.T).T

  #come back from homogeneous coordinates
  pc_transformed = pc_transformed_hom[:, :3]

  #return a point cloud (matrix Nx3)
  return pc_transformed

#compute registration error using eucledian distance
def compute_registration_error(source_pc, target_pc):

  registration_error = 0

  #for each couple of points
  for p1,p2 in zip(source_pc, target_pc):

    #compute the distance
    registration_error+=euclidean_distance(p1,p2)

  #compute the minimum distance
  registration_error/=len(source_pc)

  #return a float
  return registration_error

#ICP algorithm: execute the step to find a transformation between two point clouds and apply it
def icp_registration(source_pc, source_curvatures, target_pc, target_curvatures, correlation_method=None):

  #Find correlation between nearest points of the two point cloud with similar curvatures
  new_source_pc, new_target_pc = correlation_method(source_pc, source_curvatures, target_pc, target_curvatures)

  #correspondeces found
  if new_source_pc is not None and new_target_pc is not None:

    #Compute transformation matrix
    T = compute_transformation_matrix(new_source_pc, new_target_pc)

    #Transform the source point cloud
    transformed_source_pc = point_cloud_transformation(T, source_pc)

    #Registration error computation
    registration_error = compute_registration_error(new_source_pc, new_target_pc)
    registration_error = np.round(registration_error, 7)
    print("Registration Error: "+str(registration_error))

    #return a point cloud (Nx3), the registration error (float), a transformation matrix (3x3)
    return transformed_source_pc, registration_error, T

  #correspondences not found
  else:
    return None, None, None

#Execute the ICP for a number of itarations, plotting the results
def execute_icp(source_pc, source_curvatures, terget_pc, target_curvatures, iterations=10, correlation_method=find_correlations):

  #variables to manage the registration error
  patient = 5
  iter = 0
  registration_error_list = []

  #plot initil pose of both point clouds
  print("Initial Pose:")
  plot_registration(source_pc, terget_pc)

  #first iteration, execution of ICP
  print("Iteration 1:\n")
  source_pc_transformed, registration_error, T = icp_registration(source_pc, source_curvatures, terget_pc, target_curvatures, correlation_method)

  #Exit if no transformation matrix founds
  if registration_error is None:
    print("It is not possible to find any transformations")
    return [0,0,0,0,0], None

  #save registration error, plot registrations and save error and matrix found
  registration_error_list.append(registration_error)
  plot_registration(source_pc_transformed, terget_pc)
  max_error = registration_error
  T_min = T

  #continue the iterations from second go on
  for i in range(2,iterations-1):

    #new execution of ICP
    print("Iteration "+str(i)+":\n")
    source_pc_transformed, registration_error, T = icp_registration(source_pc_transformed, source_curvatures, terget_pc, target_curvatures, correlation_method)

    #no more transformation matrix found
    if registration_error is None:
      print("It is not possible to find other transformations")
      break

    #registraton error dercease
    elif registration_error < max_error:
      max_error = registration_error
      T_min = T
      iter = 0

    #registration error increase
    else: 
      iter+=1

    #save registration error and plot registration of this iteration
    registration_error_list.append(registration_error)
    plot_registration(source_pc_transformed, terget_pc)

    #registration error not decrease for a certain number of iterations
    if iter > patient:
      print("Stopped because the registration error don't improve anymore")
      break

  #print iteration and registration error with it is minimum
  print(f"\n\nDone {str(i)} iterations with minimun registration error: {str(min(registration_error_list))}\n\n")

  #return a list of registrations errors (list of float) and the matrix (3x3) where it is minimum
  return registration_error_list, T_min
