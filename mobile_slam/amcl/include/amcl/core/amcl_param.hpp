#ifndef AMCL_AMCL_CORE_AMCL_PARAM_HPP_
#define AMCL_AMCL_CORE_AMCL_PARAM_HPP_

namespace amcl
{
struct AmclParam
{
  double laser_min_range, laser_max_range;
  int max_beams;
  double alpha1, alpha2, alpha3, alpha4, alpha5;
  double z_hit, z_short, z_max, z_rand, sigma_hit, lambda_short;

  bool do_beamskip;
  double beam_skip_distance, beam_skip_threshold, beam_skip_error_threshold;
  double laser_likelihood_max_dist;

  int min_particles, max_particles;
  double alpha_slow, alpha_fast;
  double pf_err, pf_z;

  double dist_thresh, angle_thresh, time_thresh;

  int resample_interval;
  bool selective_resampling;

  laser_model_t laser_model_type;
  odom_model_t odom_model_type;

  double init_pose[3];
  double init_cov[3];

  bool force_update_after_initialpose = false;
  bool force_update_after_set_map = false;
  bool force_update = false;

  bool use_scan_matching = true;
  bool use_ekf = false;

  // debug param
  bool output_mean_pose = false;
};

} // namespace amcl

#endif // AMCL_AMCL_CORE_AMCL_PARAM_HPP_