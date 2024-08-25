#ifndef POS_CONTROLLER_HPP_
#define POS_CONTROLLER_HPP_

#include "MotionTrajectory.hpp"
#include "MuJoCoInterface.hpp"
#include "RobotLeg.hpp"

template <typename T>
class TrackingController
{
  private:
    RobotLeg<T> & robot_;
    Vec2<T> kp_[4], kd_[4];
    Vec2<T> force_rw_des_[4];

    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr_;
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr_;

  public:
    TrackingController(RobotLeg<T> & robot);

    void get_traj_pointer(
        std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr,
        std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr);

    void joint_HAA_control();
    void RW_posPD_control();




};


#endif  // POS_CONTROLLER_HPP_
