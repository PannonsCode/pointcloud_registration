import open3d as o3d
import numpy as np
from scipy.spatial import KDTree
from sklearn.decomposition import PCA

def read_point_cloud(pth):

  # Load the ply file
  plydata = o3d.io.read_point_cloud(pth)
  
  #return a array of points with coordinates(x,y,z)
  return np.asarray(plydata.points) #matrix Nx3

#function to reduce the dimension of a point cloud
def reduce_point_cloud(point_cloud, reduction_factor):

    #Construct a KDTree from the point cloud
    tree = KDTree(point_cloud)

    #reduce the number of points by a reduction_factor
    num_points_to_keep = int(len(point_cloud) * reduction_factor)
    indices_to_keep = np.random.choice(len(point_cloud), num_points_to_keep, replace=False)
    reduced_point_cloud = point_cloud[indices_to_keep]

    return reduced_point_cloud
