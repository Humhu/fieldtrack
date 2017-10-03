'''Functions to parse out data from IMU calibration bag files
'''

import rosbag
import numpy as np
import scipy.interpolate as spi
import tf.transformations as tform
import pdb

grav_mag = 9.81

def parse_imu_msg(msg):
    '''Parse a ROS IMU message
    '''
    gyro = [msg.angular_velocity.x,
            msg.angular_velocity.y, msg.angular_velocity.z]
    xl = [msg.linear_acceleration.x,
          msg.linear_acceleration.y, msg.linear_acceleration.z]
    ori_q = [msg.orientation.x, msg.orientation.y, 
             msg.orientation.z, msg.orientation.w]
    ori_R = tform.quaternion_matrix(ori_q)[0:3, 0:3]
    # euler = tform.euler_from_matrix(ori_R, axes='rzyx')
    grav = np.dot(ori_R.T, np.array([0, 0, -grav_mag]))

    return msg.header.stamp.to_sec(), gyro, xl, grav, ori_q


def parse_twist_msg(msg, t):
    '''Parse a ROS Twist or TwistStamped message
    '''
    def parse_twist(twist):
        return [twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z]

    if hasattr(msg, 'header'):
        # Should be a twist_stamped
        return msg.header.stamp.to_sec(), parse_twist(msg.twist)
    else:
        # Should be a twist
        return t.to_sec(), parse_twist(msg)


def parse_bag(path, imu_topic, vel_topic):
    '''Reads out data from a bag file
    '''
    bag = rosbag.Bag(path, 'r')

    # Check topics existence
    topics = bag.get_type_and_topic_info().topics
    if imu_topic not in topics:
        raise RuntimeError('Could not find topic %s in bag' % imu_topic)
    if vel_topic is not None and vel_topic not in topics:
        raise RuntimeError('Could not find topic %s in bag' % vel_topic)

    # Run through bag and read messages
    imu_t = []
    gyro_w = []
    xl_a = []
    grav = []
    body_t = []
    body_val = []
    ori_val = []
    for topic, msg, t in bag.read_messages():

        if topic == imu_topic:
            stamp, gyro, xl, g, ori = parse_imu_msg(msg)
            imu_t.append(stamp)
            gyro_w.append(gyro)
            xl_a.append(xl)
            grav.append(g)
            ori_val.append(ori)

        elif topic == vel_topic:
            stamp, vel = parse_twist_msg(msg, t)
            body_t.append(stamp)
            body_val.append(vel)

    data = {'imu_t': np.array(imu_t), 'gyro_w': np.array(gyro_w),
            'xl_a': np.array(xl_a), 'grav': np.array(grav), 'ori': np.array(ori_val)}
    if vel_topic is not None:
        data['body_t'] = np.array(body_t)
        data['body_val'] = np.array(body_val)
    else:
        data['body_t'] = data['imu_t']
        data['body_val'] = np.zeros((len(data['imu_t']), 6))
    return data

def intersect_data(data, use_imu_t=True):
    '''Computes the intersection of body velocity and IMU data
    '''
    if use_imu_t:
        interpolator = spi.interp1d(x=data['body_t'],
                                    y=data['body_val'],
                                    axis=0, bounds_error=False)

        imu_t = data['imu_t']
        gyro_w = data['gyro_w']
        xl_a = data['xl_a']
        grav = data['grav']
        body_t = data['imu_t']
        body_val = interpolator(body_t)
    else:
        # NOTE Don't have a good way to interpolate the quaternions w/o exponential map
        gyro_interpolator = spi.interp1d(x=data['imu_t'],
            y=data['gyro_w'], axis=0, bounds_error=False)
        xl_interpolator = spi.interp1d(x=data['imu_t'], y = data['xl_a'], axis = 0, bounds_error = False)
        grav_interpolator = spi.interp1d(x=data['imu_t'], y=data['grav'], axis=0, bounds_error=False)

        imu_t=data['body_t']
        gyro_w=gyro_interpolator(imu_t)
        xl_a=xl_interpolator(imu_t)
        grav = grav_interpolator(imu_t)
        body_t=data['body_t']
        body_val=data['body_val']

    # Find valid intersection of data
    start_time=max((np.min(imu_t), np.min(body_t)))
    end_time=min((np.max(imu_t), np.max(body_t)))
    t_valid=np.logical_and(imu_t >= start_time, imu_t <= end_time)
    body_valid=np.logical_not(np.any(np.isnan(body_val), axis=1))
    imu_valid=np.logical_not(np.any(np.isnan(gyro_w), axis=1))

    base_mask=np.logical_and(np.logical_and(t_valid, body_valid), imu_valid)
    
    data['times'] = imu_t[base_mask]
    data['gyro_w'] = gyro_w[base_mask]
    data['xl_a'] = xl_a[base_mask]
    data['grav'] = grav[base_mask]
    data['body_val'] = body_val[base_mask]

def parse_bag_data(path, imu_topic, vel_topic=None, use_imu_t=True):
    '''Parses and preprocesses data from a rosbag file
    '''
    data = parse_bag(path, imu_topic, vel_topic)

    # if vel_topic is not None:
    intersect_data(data, use_imu_t)

    # Numerically compute accelerations and break out w, a
    data['body_w'] = data['body_val'][:, 3:6]
    body_v = data['body_val'][:, 0:3]
    body_a=(np.diff(body_v, axis=0).T / np.diff(data['times'])).T
    data['body_a']=np.vstack((body_a, np.reshape(body_a[-1], (1, -1))))
    return data
