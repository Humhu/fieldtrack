import rospy
from sensor_msgs.msg import Imu
from um7.srv import Reset, ResetRequest

def parse_vec3(msg):
    return [msg.x, msg.y, msg.z]

def parse_imu_msg(msg):
    gyro = parse_vec3(msg.angular_velocity)
    xl = parse_vec3(msg.linear_acceleration)
    return gyro, xl

class ImuResetMonitor(object):
    '''Monitors an IMU topic and resets orientation when stationary
    '''
    
    def __init__(self):
        self.imu_sub = rospy.Subscriber( 'imu', Imu, self.imu_callback )
        reset_topic = rospy.get_param( '~reset_service' )
        self.reset_srv = rospy.ServiceProxy( reset_topic, Reset )

        self.stationary_time = rospy.get_param('~stationary_time', 1.0)
        self.reset_period = rospy.get_param('~time_between_resets', 1.0)
        self.last_reset_time = rospy.Time.now()

    def reset(self):
        req = ResetRequest()
        req.zero_gyros = True
        req.reset_ekf = True
        try:
            self.reset_srv(req)
        except rospy.ServiceException:
            rospy.logerr('Could not reset IMU')

    def imu_callback(self, msg):
        gyro, xl = parse_imu_msg(msg)

        # Test IMU data for stationarity

if __name__ == '__main__':
    rospy.init('imu_reset_monitor')