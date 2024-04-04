# pointcloud_registration

This project refer to the registration of two point clouds through the 
comparison of the curvature feature, using the Iterative Closest Point.
All the project has been developed through Google Colab, then a python 
file has been created to execute the project also in the local 
environment.

The files include:
- “EAI1_VisualPerceptionProject.ipynb : the notebook used to develop the 
project
- “requirements.txt”: a file containing the external libraries to be 
installed to execute the project locally
- “point_cloud_registration.py”: the main file to execute the project
- "read_data.py: containing" the functions to read the data
- “neighboors.py" containing the functions to find k neighboors of a point, 
its normal and the variance of a set of point
- “curvatures.py”: containing the functions to compute the curvatures with 
different methods 
- “plot_utility.py" : containing the functions to plot graphics 
- “registration.py"  containing the functions for the registrations of 
point clouds (Iterative Closest Point (ICP) is implemented) 
- “comands.json” : a file containing some parameters to execute the 
registration algorithm. In particular the field that can e setted up are: 

  -  “pth_source": a path to a point cloud (file.ply) 
	-  “pth_target": a path to a point cloud (file.ply) 
	-  “reduce_pc":  {“flag" :true/false if you want or not reduce the number of points in the point clouds 
						      "reduction_factor": a float in the range [0,1], the percentage of reduction }, 
	-  “compute_curvatures":  {“flag": true/false if you want compute the curvatures during execution or read them (pre-computed) from files 
								       “curv_source" :  a path (file.txt)					  		
									   “curv_target" : .a path (file.txt) } 
	-  “curvature_method": the method to compute curvatures, choose between [ "umbrella" , "umbrella2" , "umbrella3" , "surface_variance" ] 
	-  “k_neighboors" : an integer, the number of neighbors to use to compute curvatures 
	-  “add_noise":  {"flag": true/false if you want add or not the noise 
						      “mean" : list of 3 elements ex. [0,0,0] indicating the mean value of the gaussian noise 
						      "variance : a float indicating the variance of the gaussian noise } 
	-  correlation_method : method to find correlations between points, choose between [ "find_correlations" , "find_correlations2" ] 
	-  iterations : an integer, the max number of the iterations for the algorithm  
 
- a folder  “data”  containing the data that is possible to use (or where to add external data), it contains the subfolders: 
	-  “bunny “: with point clouds and curvatures  
	-  “armadillo”  : with point clouds and curvatures  
	-  “dragon” : with point clouds and curvatures 
  The paths requested in the comnds.json file can be written as  './object/file',
  where object is one between the three subfolder and file is a file contained into it.

- The file 'report.pdf' contains all the resume on work done
 
To execute locally: 
- Be sure to have installed the libraries in the file requirements.txt 
- Be sure to be in the folder code 

- Lunch from terminal:

			> python3 point_cloud_registration.py comands.json  
			
- Oss.: For each iteration of the algorithm, a window with the plot of the 
registration will be open, to continue next iteration close that window
