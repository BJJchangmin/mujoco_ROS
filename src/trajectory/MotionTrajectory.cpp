#include "MotionTrajectory.hpp"

#include <cmath>

using namespace std;

template <typename T>
MotionTrajectory<T>::MotionTrajectory()
  : t_(0.0),
    T_pause_(1.0),
    T_crouch_(1.0),
    T_land_(0.5),
    T_recover_(0.5),
    h_max_(0.5),
    h_min_(0.2),
    h_home_(0.32355),
    dthr_init_(0.0)
{
    state_ = 0.0;

    foot_traj_ptr_ = std::make_shared<DesiredFootTrajectory>();
    joint_traj_ptr_ = std::make_shared<DesiredJointTrajectory>();
}

template <typename T>
void MotionTrajectory<T>::stance_test()
{
  /**
   * @brief Stance_Test
   * @param joint_traj_ptr_ for HAA fix
   * @param foot_traj_ptr_ for foot trajectory
   */

  for (size_t i = 0; i < 4; i++)
  {
    //**RW control */
    foot_traj_ptr_->foot_pos_rw_des_[i] << h_home_,M_PI / 2;

    //** Joint Velocity Control for HAA */
    joint_traj_ptr_->joint_vel_des_[i][0]= 0.0;
    joint_traj_ptr_->joint_pos_des_[i][0]= 0.0;

  }

}

template <typename T>
std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> MotionTrajectory<T>::get_foot_traj_ptr()
{
  return foot_traj_ptr_;
}

template <typename T>
std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> MotionTrajectory<T>::get_joint_traj_ptr()
{
  return joint_traj_ptr_;
}




template class MotionTrajectory<float>;
template class MotionTrajectory<double>;

