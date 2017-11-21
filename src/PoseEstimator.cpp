#include "fieldtrack/PoseEstimator.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/random/MultivariateGaussian.hpp"

#include <boost/foreach.hpp>

namespace argus
{
PoseEstimator::PoseEstimator() {}

// TODO Support 2D mode
void PoseEstimator::Initialize( ros::NodeHandle& ph,
                                bool useVel,
                                ExtrinsicsInterface::Ptr extrinsics,
                                double velBuffLen )
{
	GetParamRequired( ph, "reference_frame", _referenceFrame );
	GetParamRequired( ph, "body_frame", _bodyFrame );
	GetParam( ph, "log_likelihood_threshold",
	          _logLikelihoodThreshold,
	          -std::numeric_limits<double>::infinity() );
	GetParam( ph, "max_entropy_threshold",
	          _maxEntropyThreshold,
	          std::numeric_limits<double>::infinity() );
	GetParam( ph, "two_dimensional", _twoDimensional, false );

	GetParam( ph, "initial_pose", _initialPose, PoseSE3() );
	GetParamRequired( ph, "initial_covariance", _initialCovariance );
	_filter.Initialize( _initialPose, _initialCovariance );

	if( !useVel )
	{
		GetParamRequired( ph, "transition_covariance", _transCovRate );
	}
	_enableVelocity = useVel;

	// Parse all update sources
	YAML::Node updateSources;
	GetParamRequired( ph, "update_sources", updateSources );
	YAML::Node::const_iterator iter;
	for( iter = updateSources.begin(); iter != updateSources.end(); iter++ )
	{
		const std::string& sourceName = iter->first.as<std::string>();
		ros::NodeHandle sh = ph.resolveName( "update_sources/" + sourceName );
		if( _sourceRegistry.count( sourceName ) > 0 )
		{
			throw std::invalid_argument( "Source " + sourceName + " already registered!" );
		}
		_sourceRegistry[sourceName].Initialize( sh,
		                                        _twoDimensional,
		                                        _referenceFrame,
		                                        _bodyFrame,
		                                        extrinsics );
	}

	_velocityIntegrator.SetMaxBuffLen( velBuffLen );
}

void PoseEstimator::BufferVelocity( const ros::Time& time,
                                    const PoseSE3::TangentVector& vel,
                                    const PoseSE3::CovarianceMatrix& cov )
{
	if( time < GetFilterTime() )
	{
		ROS_WARN_STREAM_THROTTLE( 30, "Velocity info from time " << time <<
		                 " before filter time " << GetFilterTime() );
		return;
	}
	_velocityIntegrator.BufferInfo( time.toSec(), vel, cov );
}

nav_msgs::Odometry PoseEstimator::GetOdom() const
{
	nav_msgs::Odometry msg;
	msg.header.frame_id = _referenceFrame;
	msg.child_frame_id = _bodyFrame;
	msg.header.stamp = GetFilterTime();
	msg.pose.pose = PoseToMsg( _filter.GetState() );
	SerializeMatrix( _filter.GetCovariance(), msg.pose.covariance );

	msg.twist.twist = TangentToMsg( _velocityIntegrator.GetLatestVelocity() );
	SerializeMatrix( _velocityIntegrator.GetLatestCovariance(), msg.twist.covariance );

	return msg;
}

geometry_msgs::PoseStamped PoseEstimator::GetPose() const
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = _bodyFrame;
	msg.header.stamp = GetFilterTime();
	msg.pose = PoseToMsg( _filter.GetState() );
	return msg;
}

geometry_msgs::PoseWithCovarianceStamped PoseEstimator::GetPoseWithCovariance() const
{
	geometry_msgs::PoseWithCovarianceStamped msg;
	msg.header.frame_id = _bodyFrame;
	msg.header.stamp = GetFilterTime();
	msg.pose.pose = PoseToMsg( _filter.GetState() );
	SerializeMatrix( _filter.GetCovariance(), msg.pose.covariance );
	return msg;
}

// CovarianceModel::Ptr PoseEstimator::InitTransCovModel() const
// {
//  TimeScaledCovariance::Ptr cov = std::make_shared<TimeScaledCovariance>();
//  cov->Initialize( _transCovRate );
//  return cov;
// }

// std::unordered_map<std::string, CovarianceModel::Ptr>
// PoseEstimator::InitObsCovModels() const
// {
//  std::unordered_map<std::string, CovarianceModel::Ptr> out;
//  typedef SourceRegistry::value_type Item;
//  BOOST_FOREACH( const Item &item, _sourceRegistry )
//  {
//      const std::string& name = item.first;
//      const PoseSourceManager& manager = item.second;
//      out[name] = manager.InitializeModel();
//  }
//  return out;
// }

// void PoseEstimator::SetTransCovModel( const CovarianceModel& model )
// {
//  // TODO Different transition covariance modes
//  try
//  {
//      const FixedCovariance& mod = dynamic_cast<const FixedCovariance&>( model );
//      _transCovRate = mod.GetValue();
//      ROS_INFO_STREAM( "Transition covariance rate updated to: " << std::endl << _transCovRate );
//  }
//  catch( std::bad_cast& e )
//  {
//      throw std::invalid_argument( "Transition cov type mismatch: " + std::string( e.what() ) );
//  }
// }

// void PoseEstimator::SetObsCovModel( const std::string& name,
//                                     const CovarianceModel& model )
// {
//  if( _sourceRegistry.count( name ) == 0 )
//  {
//      throw std::invalid_argument( "Source " + name + " not registered!" );
//  }
//  _sourceRegistry[name].SetModel( model );
// }

void PoseEstimator::ResetDerived( const ros::Time& time,
                                  const VectorType& state,
                                  const MatrixType& cov )
{
	PoseSE3 initPose = ( state.size() == 0 ) ? _initialPose : PoseSE3( state );
	PoseSE3::CovarianceMatrix initCov = ( cov.size() == 0 ) ? _initialCovariance :
	                                    PoseSE3::CovarianceMatrix( cov );

	_filter.Initialize( initPose, initCov );

	// Reset all observation covariance adapters
	typedef SourceRegistry::value_type Item;
	BOOST_FOREACH( Item & item, _sourceRegistry )
	{
		item.second.Reset();
	}

	_velocityIntegrator.Reset();
}

PredictInfo PoseEstimator::PredictUntil( const ros::Time& until )
{
	PoseSE3 displacement;
	PoseSE3::CovarianceMatrix covariance;
	double dt = (until - GetFilterTime() ).toSec();
	if( _enableVelocity )
	{
		covariance.setZero();
		if( !_velocityIntegrator.Integrate( GetFilterTime().toSec(),
											until.toSec(),
											displacement,
											covariance,
											true ) )
		{
			ROS_WARN_STREAM_THROTTLE( 30, "Could not integrate velocity from to " << until );
		}
	}
	else
	{
		covariance = _transCovRate * dt;
	}
	PredictInfo info = _filter.Predict( displacement, covariance );
	info.step_dt = dt;
	return info;
}

bool PoseEstimator::ProcessMessage( const std::string& source,
                                    const ObservationMessage& msg,
                                    UpdateInfo& info )
{
	PoseSourceManager& manager = _sourceRegistry.at( source );
	PoseObservation obs = boost::apply_visitor( manager, msg );

	// Check observation likelihood
	// TODO HACK!
	VectorType v = PoseSE3::Log( obs.pose.Inverse() * _filter.GetState() );
	MatrixType V = _filter.GetCovariance() + obs.covariance;
	double ll = GaussianLogPdf( V, v );
	if( !manager.CheckLogLikelihood( ll ) )
	{
		ROS_WARN_STREAM( "Rejecting observation from " <<
		                 source << " with log likelihood " << ll );
		return false;
	}

	info = _filter.Update( obs.pose, obs.covariance );
	info.time = GetFilterTime();
	info.frameId = source;

	manager.Update( info );
	return true;
}

void PoseEstimator::CheckFilter()
{
	double entropy = GaussianEntropy( _filter.GetCovariance() );
	if( entropy > _maxEntropyThreshold )
	{
		ROS_WARN_STREAM_THROTTLE( 1, "Filter entropy: " << entropy << " greater than max: " <<
		                 _maxEntropyThreshold << " Resetting filter..." );
		Reset( GetFilterTime() );
	}
}
}