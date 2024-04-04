import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#plot the query point, its neighboors and its normal
def plot_points(query_point, neighboors, normal, title=""):

  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  ax.scatter(neighboors[:, 0], neighboors[:, 1], neighboors[:, 2], c='r')
  ax.scatter(query_point[0], query_point[1], query_point[2], c='b')
  ax.quiver(query_point[0], query_point[1], query_point[2], normal[0], normal[1], normal[2], length=0.001, color='g')
  ax.set_title(title)
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax .set_zlabel('Z')
  plt.show()

#plot all the points of the point cloud
def plot_point_cloud(pc, elev=90, azim=270, title=""):

  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  ax.scatter(pc[:, 0], pc[:, 1], pc[:, 2])
  ax.set_title(title)
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  if elev is not None and azim is not None:
    ax.view_init(elev, azim)
  plt.show()

#plot the aligement of two point clouds
def plot_registration(source_pc, target_pc, elev=90, azim=270):

  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  ax.scatter(target_pc[:, 0], target_pc[:, 1], target_pc[:, 2], c='r')
  ax.scatter(source_pc[:, 0], source_pc[:, 1], source_pc[:, 2], c='b')
  #ax.set_title("Registration")
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.view_init(elev, azim)
  plt.show()

#plot the points with a level of intesity corresponding to the value of the curvature
def plot_curvatures(points, curvature_values1, curvature_values2):
  fig = plt.figure(figsize=(12, 6))
  ax1 = fig.add_subplot(121, projection='3d')
  ax2 = fig.add_subplot(122, projection='3d')
  cm = plt.colormaps.get_cmap('RdYlBu') #colormap
  sc1 = ax1.scatter(points[:, 0], points[:, 1], points[:, 2], c=curvature_values1, cmap=cm)
  sc2 = ax2.scatter(points[:, 0], points[:, 1], points[:, 2], c=curvature_values2, cmap=cm)
  fig.colorbar(sc1, ax=[ax1,ax2])
  ax1.set_title('Surface Variance')
  ax2.set_title('Umbrella Method')
  ax1.view_init(elev=90, azim=270)
  ax2.view_init(elev=90, azim=270)
  plt.show()

#plot the tren of the registration error
def plot_registration_errors(data):

  x = range(len(data)) 
  y = data

  fig = plt.figure(figsize=(4, 4))
  plt.plot(x, y) 
  plt.xlabel('Iterations')  
  plt.ylabel('Registration Error')
  plt.title('Registration Error Trend') 
  plt.grid(True) 
  plt.show()
