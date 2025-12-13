#include "pod.h"

#include <cmath>

#include "api.h"

double OdomPod::get_corrected_inches() {
  return raw_inches + correction_inches;
}

double OdomPod::get_raw_inches() { return raw_inches; }
double OdomPod::get_corrected_delta_inches() {
  return get_raw_delta_inches() + get_correction_delta_inches();
}
double OdomPod::get_raw_delta_inches() { return raw_inches - prev_raw_inches; }
double OdomPod::get_correction_delta_inches() {
  return correction_inches - prev_correction_inches;
}

void OdomPod::update(int sensor_centidegrees, double delta_theta_radians) {
  prev_raw_inches = raw_inches;
  prev_correction_inches = correction_inches;
  raw_inches =
      (sensor_centidegrees / 100.0) * (wheel_diameter_inches * M_PI) / 360.0;
  double correction_factor =
      (delta_theta_radians >= 0) ? correction_clock : correction_counterclock;
  correction_inches += correction_factor * delta_theta_radians;
}