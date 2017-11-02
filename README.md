# fieldtrack
Fieldtrack is a package for **field** robot **tracking** and state estimation in the ROS [argus](https://github.com/Humhu/argus) framework. This package focuses on a manifold pose estimation Kalman filter and a specifiable-order velocity estimation Kalman filter. Both filters use a buffered-cloning architecture to handle delayed and out-of-order observations exactly.

# Dependencies
* ROS: Only tested with ROS Indigo and Jade, but should work with little effort in other versions as well
* [argus_utils](https://github.com/Humhu/argus_utils): A utilities package for argus packages

# Design Concepts
## Buffered-Cloning Filters
Filters combining information from multiple observation sources must be able to handle out-of-order and delayed data. This occurs frequently with processing-heavy and/or high-latency components, such as vision or laser-based localization algorithms. 

One class of approaches is to "reverse" the filter in time to apply the delayed observation, and then play the filter forward again, but this is often tricky to implement reliably. Alternatively, the filter estimate can be delayed or "lagged" to ensure enough time for all data to be received and sorted, but this can be undesirable for high-rate control. 

The *buffered-cloning* approach is similar to the lagged approach: the filter keeps a fixed time-length buffer of observations corresponding to the desired lag and processes observations as they expire out of the buffer. To query the state at the current time, the filter and its buffer is cloned and the clone processes the entire buffer. This way, all currently received data is used to produce the current state estimate, but delayed data can be added afterwards so long as it does not exceed the buffer lag. Observations are applied in order and exactly without any reversing, and implementation is straightforward, requiring only that the filter and buffer be cloneable (copy-constructable in C parlance).

For more details, refer to the base [BufferedEstimator](https://github.com/Humhu/fieldtrack/blob/master/include/fieldtrack/BufferedEstimator.h) class.

## Manifold Kalman Filter
A robot's (or any object's) position and orientation, or *pose*, in the physical world can be represented in a variety of ways. Most common representations use Cartesian (X-Y-Z) coordinates for position and either Euler angles (yaw-pitch-roll) or quaternions for orientation. This results in a 6-dimensional parameterization for Euler angle poses and 7-dimensional for quaternion poses. However, these pose representations are not simply vectors: Euler angles are rotations and thus must wrap, while quaternions must be normalized. As a result, using a Kalman filter to track pose is not straightforward.

More specifically, we can understand pose as laying in a *manifold*, or a subspace within the 6 or 7-dimensional parameterization vector space. Without getting too deep into the math, we can perform the difference and weighted average operations required for Kalman filtering in the manifold tangent space. For poses, this means we operate in the space of body velocities, which is quite sane to think about. For a general reference, see from Tyagi and Davis [this paper](http://web.cse.ohio-state.edu/~davis.1719/Publications/cvpr08-b.pdf) and Ethan Eade's excellent [tutorials](http://ethaneade.com/).

One further issue not discussed in the aforementioned references is the matter of *noise handedness*, which is a term I just made up. In short, we represent pose noise as a random pose perturbation applied either on the left or right-hand side of the pose itself, hence the terminology. For the convention of pose being target-to-observer, we use a left-handed noise convention in fieldtrack, meaning noise is applied after the pose.

## Noise Covariance Learning
Previously fieldtrack filters could use the Kalman filter recurrence structure to optimize the system covariance properties online. This capability is currently deprecated as a result of separating fieldtrack from the main argus package. There are currently no plans to update it.

## Adaptive Kalman Filtering
Kalman filters require the observation noise covariance to be specified for each observation source, but picking an appropriate covariance can often be difficult. Adaptive Kalman filters estimate the noise covariance over a short time window, ensuring that the different observation noise sources will be approximately in scale relative to the transition noise covariance. Refer to [this (paywalled) paper](https://link.springer.com/article/10.1007%2Fs001900050236?LI=true) by Mohamed and Schwarz for a nice reference.

## Transition Covariance Rates
The fieldtrack filters operate in continuous time. Thus, instead of using a fixed transition covariance, we use transition covariance *rates* that are scaled linearly with the transition time length.

# Usage
## Specifying Matrices
Fieldtrack uses the argus_utils parameter reading tools for loading covariance matrices. In short, diagonal N by N matrices can be specified with an array of N floats (inf and nan are parseable as well), and dense N by N matrices can be specified row-major with an array of N * N floats. For more details, refer to [argus_utils](https://github.com/Humhu/argus_utils).

## Specifying Update Sources
Observation update sources for fieldtrack filters are specified with nested blocks of private parameters in ~update_sources. The format is illustrated below.
* `~update_sources/[name]`: The block entrypoint is also the name of the source for debug purposes
* `~update_sources/[name]/type`: ('pose', 'pose_with_cov', 'transform', 'imu', 'twist', or 'twist_with_cov') The topic message type
* `~update_sources/[name]/topic`: (string) The message topic path from the node namespace (not private)
* `~update_sources/[name]/buffer_size`: (unsigned int, default 10) The subscription topic buffer length
* `~update_sources/[name]/min_log_likelihood`: (float, default -inf) Likelihood gating threshold for observations

Observation covariances are populated in one of three ways: pass-through, fixed, or adaptive. Each mode and their parameters are detailed below:

* `~update_sources/[name]/mode`: ('pass', 'fixed', 'adaptive') The observation covariance type

### Pass-Through
Pass-through mode uses the covariance field in a geometry_msgs/TwistStampedWithCovariance message. An exception will be thrown if this mode is used with any other message type.

Pass-through mode requires no parameters.

### Fixed
Fixed uses a specified fixed covariance for all observations.

* `~update_sources/[name]/fixed_covariance`: (array of floats parseable as matrix) The fixed covariance to use

### Adaptive
Adaptive uses an adaptive Kalman covariance estimator. Specifically, we use a stable additive residual form to estimate instantaneous covariances for each observation, and then return a time-weighted covariance. Covariances are weighted exponentially less as they age, and a prior covariance is added with a fixed exponential weight. With this model, the covariance estimate transitions from the prior rapidly to the adaptive estimate when there are enough samples, and gradually back towards the prior as the samples age.

* `~update_sources/[name]/max_window_samples`: (unsigned int) The max sample window size. Oldest samples are dropped as this size is exceeded.
* `~update_sources/[name]/min_window_samples`: (unsigned int, default 0) The min number of samples required to use the adaptive estimate versus the prior
* `~update_sources/[name]/max_sample_age`: (float) The max age of a sample in seconds, after which it is dropped
* `~update_sources/[name]/prior_cov`: (array of floats, default [1e-3]) The prior covariance term to use
* `~update_sources/[name]/prior_age`: (float, default 1.0) The fixed age in seconds to use for weighting the prior
* `~update_sources/[name]/use_diag`: (bool, default true) Whether or not to only keep the diagonal of the adaptive covariance estimate
* `~update_sources/[name]/decay_rate`: (positive float, default 1.0) The time constant used to compute exponential weights as exp( -age * decay_rate )

# Nodes
## velocity_estimator_node
This node can subscribe to a variety of topic types to estimate the body velocity of a robot. Internally the node wraps a buffered-cloning Kalman filter tracking a specifiable number of pose derivatives, i.e. a first-order filter tracks only velocity, a second-order filter tracks velocity and acceleration.

### Supported Topic Types
The velocity estimator interprets the below topic types into observations through `VelocitySourceManager`. The message header timestamps are used to order the observations, and the header frame IDs are used to transform velocities to the robot body frame with TF.

* `geometry_msgs/TwistStamped`: An observation of the robot body velocity at the frame `header/frame_id`
* `geometry_msgs/TwistStampedWithCovariance`: An observation of the robot body velocity at the frame `header/frame_id`, plus the noise covariance
* `sensor_msgs/Imu`: Angular velocity and linear acceleration observations at the frame `header/frame_id`

### Parameters
The velocity estimator is specified almost entirely through private ROS parameters for better readability.

#### Filtering
* `~body_frame`: (string) Body frame ID
* `~max_entropy_threshold`: (float, default inf) Maximum estimate entropy after which the filter is reset
* `~two_dimensional`: (bool, default false) Whether or not the velocities should be two dimensional (x, y, yaw only)
* `~filter_order`: (unsigned int) The number of velocity derivatives to track. Zero means track only velocity.
* `~initial_mean`: (n-array of float, default [zeros]) The filter velocity on initialization and after reset. Should be length 3 x (order+1) for 2D mode, 6 x (order+1) for 3D mode.
* `~initial_covariance`: (array of float parseable as matrix, default [zeros]) The filter covariance on initialization and after reset. Should be length N for diagonal matrix, or N * N for dense matrix, where N is length of initial mean (see above)
* `~transition_covariance`: (array of float parseable as matrix) The filter transition covariance rate. Same size requirements as initial_covariance.

#### Spinning
* `~update_lag`: (float) Buffer lag in seconds
* `~num_threads`: (unsigned int, default 1) Number of spinner threads to use
* `~update_rate`: (float) Update and publish rate in Hz

#### Publishing
* `~publish_odom`: (bool, default false) Whether or not to publish `nav_msgs/Odometry` messages on topic `odom`
* `~odom_buff_len`: (unsigned int, default 10) The odometry publisher buffer length
* `~publish_twist`: (bool, default false) Whether or not to publish `geometry_msgs/TwistStamped` on topic `twist`
* `~twist_buff_len`: (unsigned int, default 10) The twist publisher buffer length
* `~publish_twist_with_cov`: (bool default false) Whether or not to publish `geometry_msgs/TwistStampedWithCovariance` on topic `twist_with_cov`
* `~twist_with_cov_buff_len`: (unsigned int, default 10) The twist with covariance publisher buffer length
* `~publish_info`: (bool, default false) Whether or not to publish `argus_msgs/FilterInfo` on topic `info`
* `~info_buff_len`: (unsigned int, default 10) The info publisher buffer length

