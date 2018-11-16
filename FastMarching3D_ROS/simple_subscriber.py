#!/usr/bin/env python
import rospy
import numpy as np
import scipy.io
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def callback(ros_cloud):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", 'yea')
    # pcl_data = ros_to_pcl(ros_cloud)
    esdf_3d = ros_to_np(ros_cloud)
    scipy.io.savemat('esdf_3d_example', mdict={'esdf_3d': esdf_3d})
    print('File saved.')

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/X1/voxblox_node/esdf_pointcloud', PointCloud2, callback)
    rospy.spin()
    return

def ros_to_np(ros_cloud):
    """ Converts a ROS PointCloud2 message to a numpy N by 3 matrix array

        Args:
            ros_cloud (PointCloud2): ROS PointCloud2 message

        Returns:
            numpy Nx3 matrix array
    """
    file = open('esdf_3d.txt', 'w')
    points_list = []

    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2], data[3]])

    xdata = [row[0] for row in points_list]
    ydata = [row[1] for row in points_list]
    zdata = [row[2] for row in points_list]
    x_range = [np.min(xdata), np.max(xdata)]
    y_range = [np.min(ydata), np.max(ydata)]
    z_range = [np.min(zdata), np.max(zdata)]
    n_x = int(np.round((x_range[1] - x_range[0])/0.2) + 1)
    n_y = int(np.round((y_range[1] - y_range[0])/0.2) + 1)
    n_z = int(np.round((z_range[1] - z_range[0])/0.2) + 1)
    print('(%.1f, %.1f); (%.1f, %.1f); (%.1f, %.1f)' % (x_range[0], x_range[1], y_range[0], y_range[1], z_range[0], z_range[1]))
    print('%d, %d, %d' % (n_x, n_y, n_z))

    esdf_3d = np.zeros((n_x, n_y, n_z, 4), dtype=np.double)

    for data in points_list:
        file.write('%0.1f, %0.1f, %0.1f, %f\n' % (data[0], data[1], data[2], data[3]))
        ind_x = int(np.round((data[0] - x_range[0])/0.2))
        ind_y = int(np.round((data[1] - y_range[0])/0.2))
        ind_z = int(np.round((data[2] - z_range[0])/0.2))
        # print('x: %0.1f, y: %0.1f, z: %0.1f' % (data[0], data[1], data[2]))
        # print('ind_x: %d, ind_y: %d, ind_z: %d' % (ind_x, ind_y, ind_z))
        for i in range(0,4):
            esdf_3d[ind_x, ind_y, ind_z, i] =  data[i]

    return esdf_3d

if __name__ == '__main__':
    listener()