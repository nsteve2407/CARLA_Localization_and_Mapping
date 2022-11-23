from libcpp.vector cimport vector
from pcl cimport PointCloud_t, ShPtr_PointCloud_t, NDTSettings

cdef extern from "ndt.h":
    cdef vector[double] get_transformation_vector(ShPtr_PointCloud_t target_cloud, ShPtr_PointCloud_t input_cloud, NDTSettings settings)

def get_transformation_vector_wrapper(
    target_cloud_raw,
    input_cloud_raw,
    int iter,
    double grid_step,
    double grid_extent,
    double optim_step_x,
    double optim_step_y,
    double optim_step_theta,
    double epsilon,
    double guess_x,
    double guess_y,
    double guess_theta
):
    cdef size_t n = len(target_cloud_raw)
    cdef PointCloud_t target_cloud = PointCloud_t(1, n)
    for i in range(n):
        target_cloud.at(i).x = target_cloud_raw[i][0]
        target_cloud.at(i).y = target_cloud_raw[i][1]
        target_cloud.at(i).z = 0.

    n = len(input_cloud_raw)
    cdef PointCloud_t input_cloud = PointCloud_t(1, n)
    for i in range(n):
        input_cloud.at(i).x = input_cloud_raw[i][0]
        input_cloud.at(i).y = input_cloud_raw[i][1]
        input_cloud.at(i).z = 0.

    cdef NDTSettings settings
    settings.iter = iter
    settings.grid_step = grid_step
    settings.grid_extent = grid_extent
    settings.optim_step_x = optim_step_x
    settings.optim_step_y = optim_step_y
    settings.optim_step_theta = optim_step_theta
    settings.epsilon = epsilon
    settings.guess_x = guess_x
    settings.guess_y = guess_y
    settings.guess_theta = guess_theta

    cdef vector[double] s = get_transformation_vector(target_cloud.makeShared(), input_cloud.makeShared(), settings)
    return [s.data()[0], s.data()[1], s.data()[2], s.data()[3], s.data()[4]]
