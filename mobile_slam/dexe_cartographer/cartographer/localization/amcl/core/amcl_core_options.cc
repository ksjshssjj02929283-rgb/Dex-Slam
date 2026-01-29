#include <glog/logging.h>

#include "cartographer/localization/amcl/core/amcl_core_options.h"

namespace cartographer {
namespace localization {
namespace amcl {

proto::AMCLCoreOptions CreateAMCLCoreOptions(
  common::LuaParameterDictionary* const parameter_dictionary) {
  proto::AMCLCoreOptions options;
  options.set_min_particles(parameter_dictionary->GetInt("min_particles"));
  options.set_max_particles(parameter_dictionary->GetInt("max_particles"));
  options.set_kld_err(parameter_dictionary->GetDouble("kld_err"));
  options.set_kld_z(parameter_dictionary->GetDouble("kld_z"));
  options.set_recovery_alpha_slow(parameter_dictionary->GetDouble("recovery_alpha_slow"));
  options.set_recovery_alpha_fast(parameter_dictionary->GetDouble("recovery_alpha_fast"));
  options.set_resample_interval(parameter_dictionary->GetInt("resample_interval"));

  options.set_max_beams(parameter_dictionary->GetDouble("max_beams"));

  options.set_odom_model_type(parameter_dictionary->GetString("odom_model_type"));
  options.set_odom_alpha1(parameter_dictionary->GetDouble("odom_alpha1"));
  options.set_odom_alpha2(parameter_dictionary->GetDouble("odom_alpha2"));
  options.set_odom_alpha3(parameter_dictionary->GetDouble("odom_alpha3"));
  options.set_odom_alpha4(parameter_dictionary->GetDouble("odom_alpha4"));
  options.set_odom_alpha5(parameter_dictionary->GetDouble("odom_alpha5"));

  options.set_initial_cov_xx(parameter_dictionary->GetDouble("initial_cov_xx"));
  options.set_initial_cov_yy(parameter_dictionary->GetDouble("initial_cov_yy"));
  options.set_initial_cov_aa(parameter_dictionary->GetDouble("initial_cov_aa"));

  // LOG(INFO) << "min_particles: " << options.min_particles();
  // LOG(INFO) << "max_particles: " << options.max_particles();
  // LOG(INFO) << "kld_err: " << options.kld_err();
  // LOG(INFO) << "kld_z: " << options.kld_z();
  // LOG(INFO) << "recovery_alpha_slow: " << options.recovery_alpha_slow();
  // LOG(INFO) << "recovery_alpha_fast: " << options.recovery_alpha_fast();
  // LOG(INFO) << "resample_interval: " << options.resample_interval();
  // LOG(INFO) << "odom_model_type: " << options.odom_model_type();
  // LOG(INFO) << "odom_alpha1: " << options.odom_alpha1();
  // LOG(INFO) << "odom_alpha2: " << options.odom_alpha2();
  // LOG(INFO) << "odom_alpha3: " << options.odom_alpha3();
  // LOG(INFO) << "odom_alpha4: " << options.odom_alpha4();
  // LOG(INFO) << "odom_alpha5: " << options.odom_alpha5();
  // LOG(INFO) << "initial_cov_xx: " << options.initial_cov_xx();
  // LOG(INFO) << "initial_cov_yy: " << options.initial_cov_yy();
  // LOG(INFO) << "initial_cov_aa: " << options.initial_cov_aa();

  return options;
}

}  // namespace amcl
}  // namespace localization
}  // namespace cartographer
