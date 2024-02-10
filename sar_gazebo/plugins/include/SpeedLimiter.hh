#ifndef IGNITION_GAZEBO_SYSTEMS_SPEEDLIMITER_HH_
#define IGNITION_GAZEBO_SYSTEMS_SPEEDLIMITER_HH_

#include <memory>

#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration.
  class SpeedLimiterPrivate;

  class IGNITION_GAZEBO_VISIBLE SpeedLimiter
  {
    public: SpeedLimiter(bool   _hasVelocityLimits     = false,
                         bool   _hasAccelerationLimits = false,
                         bool   _hasJerkLimits         = false,
                         double _minVelocity           = 0.0,
                         double _maxVelocity           = 0.0,
                         double _minAcceleration       = 0.0,
                         double _maxAcceleration       = 0.0,
                         double _minJerk               = 0.0,
                         double _maxJerk               = 0.0);

    public: ~SpeedLimiter();

    public: double Limit(double &_v,
                         double _v0,
                         double _v1,
                         double _dt) const;

    public: double LimitVelocity(double &_v) const;

    public: double LimitAcceleration(double &_v,
                                     double _v0,
                                     double _dt) const;

    public: double LimitJerk(double &_v,
                             double _v0,
                             double _v1,
                             double _dt) const;

    private: std::unique_ptr<SpeedLimiterPrivate> dataPtr;
  };
  }
 }
 }
}

#endif