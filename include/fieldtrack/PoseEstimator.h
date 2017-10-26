#pragma once

#include "fieldtrack/BufferedEstimator.h"
#include "fieldtrack/PoseSourceManager.h"
// #include "fieldtrack/CovarianceModels.h"

#include "argus_utils/filter/PoseKalmanFilter.h"
#include "argus_utils/geometry/VelocityIntegrator.hpp"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <unordered_map>

namespace argus
{
class PoseEstimator
	: public BufferedEstimator
{
public:

	PoseEstimator();

	void Initialize( ros::NodeHandle& ph, ExtrinsicsInterface::Ptr extrinsics,
	                 double velBuffLen );

	void BufferVelocity( const ros::Time& time,
	                     const PoseSE3::TangentVector& vel,
	                     const PoseSE3::CovarianceMatrix& cov );

	nav_msgs::Odometry GetOdom() const;
	geometry_msgs::PoseStamped GetPose() const;
	geometry_msgs::PoseWithCovarianceStamped GetPoseWithCovariance() const;

	// CovarianceModel::Ptr InitTransCovModel() const;
	// std::unordered_map<std::string, CovarianceModel::Ptr> InitObsCovModels() const;
	// void SetTransCovModel( const CovarianceModel& model );
	// void SetObsCovModel( const std::string& name, const CovarianceModel& model );

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:

	PoseSE3KalmanFilter _filter;
	PoseSE3 _initialPose;
	PoseSE3::CovarianceMatrix _initialCovariance;
	PoseSE3::CovarianceMatrix _transCovRate;

	std::string _referenceFrame;
	std::string _bodyFrame;
	double _logLikelihoodThreshold;
	double _maxEntropyThreshold;

	bool _twoDimensional;

	typedef std::unordered_map<std::string, PoseSourceManager> SourceRegistry;
	SourceRegistry _sourceRegistry;

	VelocityIntegratorSE3 _velocityIntegrator;

	virtual void ResetDerived( const ros::Time& time,
	                           const VectorType& state = VectorType(),
	                           const MatrixType& cov = MatrixType() );
	virtual PredictInfo PredictUntil( const ros::Time& until );
	virtual bool ProcessMessage( const std::string& source,
	                             const ObservationMessage& msg,
	                             UpdateInfo& info );
	virtual void CheckFilter();
};
}