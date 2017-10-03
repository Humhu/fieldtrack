import numpy as np
import argparse
import scipy.optimize as spo
import se3
import matplotlib.pyplot as plt
import tf.transformations as tform
import cma

from bag_parser import parse_bag_data

gravity_mag = 9.81
xl_scale_tol = 0.1
gyro_bias_tol = 0.1


def compute_xl_scale(data):
    '''Compute a constant scale factor for all axes to correct accelerometer scaling.
    Checks to make sure the IMU is stationary on a level surface.

    Returns k so that xl_correct = k * xl
    '''
    norms = [np.linalg.norm(ai) for ai in data['xl_a']]
    norm_mean = np.mean(norms)
    norm_sd = np.std(norms)
    if norm_sd > xl_scale_tol:
        raise RuntimeError('XL magnitude has standard deviation of %f > tolerance %f' % (
            norm_sd, xl_scale_tol))
    return gravity_mag / norm_mean


def compute_gyro_bias(data):
    '''Compute a constant offset for each gyro axis to correct biases.
    Checks to make sure the IMU is stationary.

    Returns b so that w_correct = w - b
    '''
    w_sds = np.std(data['gyro_w'], axis=1)
    if np.any(w_sds > gyro_bias_tol):
        raise RuntimeError('Gyro standard deviations of %s > tolerance %f' % (
            str(w_sds), gyro_bias_tol))
    return np.mean(data['gyro_w'], axis=0)

def correct_intrinsics(data, k_xl, b_gyro):
    data['xl_a'] = data['xl_a'] * k_xl
    data['gyro_w'] = data['gyro_w'] - b_gyro

def map_transforms(x, inverse=False):
    '''Maps between 6-vector and R matrix, T vector with x-y-z-yaw-pitch-roll parameterization. Default takes IMU to body frame.
    '''
    extR = tform.euler_matrix(0, x[3], x[4], 'rzyx')
    extT = tform.translation_matrix(x[0:3])
    H = np.dot(extT, extR)
    if inverse:
        H = np.linalg.inv(H)
    return H[0:3, 0:3], H[0:3, -1]

def predict_imu(x, data):
    _, T_imu_body = map_transforms(x, inverse=False)
    R_body_imu, _ = map_transforms(x, inverse=True)
    gyro_pred = np.dot(R_body_imu, data['body_w'].T).T
    # centripetal in body frame
    centrip = np.cross(data['body_w'], np.cross(data['body_w'], T_imu_body))
    # gravity is already in IMU frame
    xl_pred = data['grav'] + np.dot(R_body_imu, (centrip + data['body_a']).T).T
    return gyro_pred, xl_pred

def predict_body(x, data):
    R_imu_body, T_imu_body = map_transforms(x, inverse=False)
    body_w = np.dot(R_imu_body, data['gyro_w'].T).T
    # centripetal in body frame to be subtracted
    centrip = np.cross(data['body_w'], np.cross(data['body_w'], T_imu_body))
    # gravity in IMU frame to be subtracted
    body_a = np.dot(R_imu_body, (data['xl_a'] - data['grav']).T).T - centrip
    return body_w, body_a

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('static_bag_path', type=str,
                        help='Path to static bag file')
    parser.add_argument('bag_path', type=str, help='Path to the bag file')
    parser.add_argument('imu_topic', type=str, help='IMU topic')
    parser.add_argument('vel_topic', type=str, help='Body velocity topic')
    parser.add_argument('-r', '--ref_frame', type=str,
                        default='', help='Extrinsics reference frame')
    parser.add_argument('-t', '--imu_times', type=bool, default=True,
                        help='Whether to use IMU or twist timestamps for interpolation')
    args = parser.parse_args()

    # First compute basic intrinsics
    # TODO Flag for imu times
    static_data = parse_bag_data(path=args.static_bag_path,
                                 imu_topic=args.imu_topic)
    k_xl = compute_xl_scale(static_data)
    b_gyro = compute_gyro_bias(static_data)
    print 'Computed accelerometer scale: %f gyro bias: %s' % (k_xl, str(b_gyro))
    correct_intrinsics(static_data, k_xl, b_gyro)
    g_residuals = static_data['xl_a'] - static_data['grav']

    # Set up optimization
    dyn_data = parse_bag_data(path=args.bag_path,
                              imu_topic=args.imu_topic,
                              vel_topic=None)
                              #vel_topic=args.vel_topic)
    correct_intrinsics(dyn_data, k_xl, b_gyro)

    def objective_func(x):
        w, a = predict_imu(x, dyn_data)
        gyro_err = w - dyn_data['gyro_w']
        xl_err = a - dyn_data['xl_a']
        return np.sum(gyro_err ** 2) + np.sum(xl_err ** 2)

    cma_options = cma.CMAOptions()
    init_x = [0, 0, 0, 3.14, 0]
    init_sds = 1.0
    init_opt = cma.CMAEvolutionStrategy(init_x, init_sds, cma_options)
    while not init_opt.stop():
        xs = init_opt.ask()
        errs = [objective_func(xi) for xi in xs]
        init_opt.tell(xs, errs)
        init_opt.logger.add()
        init_opt.disp()
    init_result = init_opt.result()

    result = spo.minimize(fun=objective_func,
                          x0=init_result[0], method='BFGS', jac=False)

    pred_w, pred_a = predict_body(result.x, dyn_data)
    plt.ion()
    plt.figure()
    colors = ['r', 'g', 'b']
    for i in range(3):
        plt.plot(pred_w[:, i], color=colors[i], linestyle='-')
        plt.plot(dyn_data['body_w'][:, i], color=colors[i], linestyle='--')
    plt.figure()
    for i in range(3):
        plt.plot(pred_a[:, i], color=colors[i], linestyle='-')
        plt.plot(dyn_data['body_a'][:, i], color=colors[i], linestyle='--')
