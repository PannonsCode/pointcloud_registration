#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  point_cloud_registration.py
#  
#  Copyright 2023 Mattia Pannone <mattiapannone@Air-di-Mattia-2.homenet.telecomitalia.it>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#

import sys
import numpy as np
from scipy.spatial import KDTree
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
import json

from read_data import *
from neighboors import *
from curvatures import *
from plot_utility import *
from registration import *

if __name__ == '__main__':
    
    args = sys.argv
    
    if len(args)<2:
        print("Error: insert a configuration file")
        exit()
    else:
        with open(args[1], 'r') as f:
            comands = json.load(f)
        
        pth_source = comands["pth_source"]
        pth_target = comands["pth_target"]
        reduce_pc = comands["reduce_pc"]["flag"]
        compute_curv = comands["compute_curvatures"]["flag"]
        curv_method = comands["curvature_method"]
        k_neighboors = comands["k_neighboors"]
        add_noise = comands["add_noise"]["flag"]
        corr_method = find_correlations if comands["correlation_method"]=="find_correlations" else find_correlations2
        iterations = comands["iterations"]
        
        pc_source = read_point_cloud(pth_source)
        pc_target = read_point_cloud(pth_target)
        
        if reduce_pc:
            reduction_factor = comands["reduce_pc"]["reduction_factor"]
            pc_source = reduce_point_cloud(pc_source, reduction_factor)
            pc_target = reduce_point_cloud(pc_target, reduction_factor)
            
        if compute_curv:
            curvatures_source = point_cloud_curvatures(pc_source, k_neighboors, method=curvature_method)
            curvatures_target = point_cloud_curvatures(pc_target, k_neighboors, method=curvature_method)
        else:
            pth_curv_source = comands["compute_curvatures"]["curv_source"]
            pth_curv_target = comands["compute_curvatures"]["curv_target"]
            curvatures_source = []
            with open(pth_curv_source, "r") as f:
                for line in f:
                    curvatures_source.append(line.strip())
            curvatures_source = [float(c) for c in curvatures_source]

            curvatures_target = []
            with open(pth_curv_target, "r") as f:
                for line in f:
                    curvatures_target.append(line.strip())
            curvatures_target = [float(c) for c in curvatures_target]
            
        if add_noise:
            mean = comands["add_noise"]["mean"]
            variance = comands["add_noise"]["variance"]
            noise = np.random.normal(mean, variance, pc2.shape)
            pc_source = pc_source + noise
            
        
        registration_error_list, _ = execute_icp(pc_source, curvatures_source, pc_target, curvatures_target, iterations, corr_method)
