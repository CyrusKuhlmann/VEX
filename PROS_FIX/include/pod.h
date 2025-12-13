
class OdomPod {
 private:
  double wheel_diameter_inches;
  double correction_clock;
  double correction_counterclock;
  double prev_raw_inches;
  double raw_inches;
  double prev_correction_inches;
  double correction_inches;

 public:
  double get_corrected_inches();
  double get_raw_inches();
  double get_corrected_delta_inches();
  double get_raw_delta_inches();
  double get_correction_delta_inches();
  void update(int sensor_centidegrees, double delta_theta_radians);
  OdomPod(double wheel_diameter_inches, double correction_clock,
          double correction_counterclock)
      : wheel_diameter_inches(wheel_diameter_inches),
        correction_clock(correction_clock),
        correction_counterclock(correction_counterclock),
        prev_raw_inches(0.0),
        raw_inches(0.0),
        prev_correction_inches(0.0),
        correction_inches(0.0) {};
};