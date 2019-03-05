from openai_ros import robot_gazebo_env
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class PR2Env(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all Robot environments.
    """

    def __init__(self):
        """Initializes a new Robot environment.
        """
        # Variables that we give through the constructor.

        # Internal Vars
        self.controllers_list = []

        self.robot_name_space = ""

        reset_controls_bool = False
        
        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        
        super(PR2Env, self).__init__(controllers_list=self.controllers_list,
                                     robot_name_space=self.robot_name_space,
                                     reset_controls=reset_controls_bool)

        self.gazebo.unpauseSim()

        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber("/base_odometry/odom", Odometry, self._odom_callback)
        rospy.Subscriber("/base_scan", LaserScan, self._laser_scan_callback)

        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self._check_publishers_connection()

        self.gazebo.pauseSim()

        rospy.logdebug("Finished TurtleBot2Env INIT...")

    # Methods needed by the RobotGazeboEnv
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        self._check_odom_ready()
        self._check_laser_scan_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_odom_ready(self):
        self.odom = None
        rospy.logdebug("Waiting for /base_odometry/odom to be READY...")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message("/base_odometry/odom", Odometry, timeout=5.0)
                rospy.logdebug("Current /base_odometry/odom READY=>")

            except:
                rospy.logerr("Current /base_odometry/odom not ready yet, retrying for getting odom")

        return self.odom

    def _check_laser_scan_ready(self):
        self.laser_scan = None
        rospy.logdebug("Waiting for /base_scan to be READY...")
        while self.laser_scan is None and not rospy.is_shutdown():
            try:
                self.laser_scan = rospy.wait_for_message("/base_scan", LaserScan, timeout=5.0)
                rospy.logdebug("Current /base_scan READY=>")

            except:
                rospy.logerr("Current /base_scan not ready yet, retrying for getting laser_scan")
        return self.laser_scan

    def _odom_callback(self, data):
        self.odom = data

    def _laser_scan_callback(self, data):
        self.laser_scan = data

    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self._cmd_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to _cmd_vel_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_cmd_vel_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()
    
    
    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
        
    # Methods that the TrainingEnvironment will need.
    # ----------------------------