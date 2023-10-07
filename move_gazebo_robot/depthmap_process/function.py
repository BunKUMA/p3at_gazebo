import numpy as np
import open3d as o3d

def changeDep480to64(np_depth):
    lines = 16
    rows , cols = np_depth.shape
    pixel_y = [round(i * int(rows/lines)) for i in range(lines)]
    mask = np.ones((rows ,cols), dtype='bool')
    mask[pixel_y,:] = False
    np_depth[mask] = 0.0
    return np_depth

def Function(depth_data):
    depth_data = changeDep480to64(depth_data)
    
    return depth_data

#   =============================================================

def cloud_16(point_cloud):
    cloud_16 = np.array([0.,0.,0.])
    lines = []
    point_cloud[:,2] += 1.7
    z_min = point_cloud[:,2].min()
    z_max = point_cloud[:,2].max()
    # 16等分
    boundaries = np.linspace(z_min, z_max, num=17)
    for boundarie in boundaries[:-1]:
        condition = (point_cloud[:,2] > boundarie) & (point_cloud[:,2] < boundarie+0.01)
        cloud_16 = np.vstack((cloud_16,point_cloud[condition]))
        lines.append(point_cloud[condition])
    print(f'len(lines):{len(lines)}')
        
        
    cloud_16 = cloud_16[1:]
    # cloud_16[:,2] -= 1.7
    print(f"cloud_16.shape:{cloud_16.shape}")
    return point_cloud

def voxel_down_sample(point_cloud):
    sparse_cloud = o3d.geometry.PointCloud()
    sparse_cloud.points = o3d.utility.Vector3dVector(point_cloud)
    voxel_size = 0.05
    voxel_cloud = sparse_cloud.voxel_down_sample(voxel_size)
    output_cloud = np.asarray(voxel_cloud.points)
    return output_cloud

def PointFunction(point_cloud):
    # cloud_16(point_cloud)
    # voxel_down_sample(point_cloud)
    return point_cloud