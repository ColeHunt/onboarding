package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
  // Motor controller objects and constants
  private static final boolean INVERT_LEFT_DRIVE = false;
  TalonSRX fl_drive_motor_;
  TalonSRX fr_drive_motor_;
  TalonSRX bl_drive_motor_;
  TalonSRX br_drive_motor_;

  // IMU sensor
  AHRS imu_;

  // Odometry objects and constants
  private static final double WHEEL_RADIUS = Units.inchesToMeters(3.0); // in inches
  private static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
  private static final double TRACK_WIDTH = Units.inchesToMeters(27.0); // in inches

  // Inputs from sensors
  double left_velocity_ = 0.0;
  double right_velocity_ = 0.0;
  double left_position_ = 0.0;
  double right_position_ = 0.0;
  double forward_command_ = 0.0;
  double turn_command_ = 0.0;
  double yaw_ = 0.0;

  // Outputs to motors
  double left_applied_percent_ = 0.0;
  double right_applied_percent_ = 0.0;

  Drivetrain() {
    // Initialize devices
    fl_drive_motor_ = new TalonSRX(1);
    bl_drive_motor_ = new TalonSRX(2);
    fr_drive_motor_ = new TalonSRX(3);
    br_drive_motor_ = new TalonSRX(4);
    imu_ = new AHRS(NavXComType.kMXP_SPI);

    // Set followers
    bl_drive_motor_.follow(fl_drive_motor_);
    br_drive_motor_.follow(fr_drive_motor_);

    // Invert motors
    fl_drive_motor_.setInverted(INVERT_LEFT_DRIVE);
    bl_drive_motor_.setInverted(INVERT_LEFT_DRIVE);
    fr_drive_motor_.setInverted(!INVERT_LEFT_DRIVE);
    br_drive_motor_.setInverted(!INVERT_LEFT_DRIVE);

    // Configure encoders
    fl_drive_motor_.setSensorPhase(true);
    fr_drive_motor_.setSensorPhase(true);
    fl_drive_motor_.setSelectedSensorPosition(0);
    fr_drive_motor_.setSelectedSensorPosition(0);

    // Zero IMU
    while (imu_.isCalibrating()) {
      Timer.delay(0.02);
    }
    imu_.zeroYaw();
  }

    /**
   * Reads all inputs from the robot and saves them to global variables
   */
  void readInputs() {
    left_velocity_ = fl_drive_motor_.getSelectedSensorVelocity() / 4096.0 * WHEEL_CIRCUMFERENCE * 10; // m/s
    right_velocity_ = fr_drive_motor_.getSelectedSensorVelocity() / 4096.0 * WHEEL_CIRCUMFERENCE * 10; // m/s
    left_position_ = fl_drive_motor_.getSelectedSensorPosition() / 4096.0 * WHEEL_CIRCUMFERENCE; // m
    right_position_ = fr_drive_motor_.getSelectedSensorPosition() / 4096.0 * WHEEL_CIRCUMFERENCE; // m
    yaw_ = -imu_.getYaw() * Math.PI / 180.0;
  }

  /**
   * Writes desired applied percent output to motors
   */
  void writeOutputs() {
    fl_drive_motor_.set(TalonSRXControlMode.PercentOutput, left_applied_percent_);
    fr_drive_motor_.set(TalonSRXControlMode.PercentOutput, right_applied_percent_);
  }

  /**
   * Outputs telemetry to dashboard
   */
  void outputTelemetry() {
    // Display odometry information to dashboard
    SmartDashboard.putNumber("Chassis Distance", getPosition());
    SmartDashboard.putNumber("Chassis Velocity", getVelocity());
    SmartDashboard.putNumber("Chassis Yaw", yaw_ * 180.0 / Math.PI);
    SmartDashboard.putNumber("Chassis Yaw Rate", getAngularVelocity() * Math.PI / 180.0);
  }

  /**
   * Zeros the odometry encoders
   */
  void zeroOdometry() {
    fl_drive_motor_.setSelectedSensorPosition(0);
    fr_drive_motor_.setSelectedSensorPosition(0);
  }

  /**
   * Calculates the traveled distance of the robot
   * @return traveled distance in meters
   */
  double getPosition() {
    return (left_position_ + right_position_) / 2.0;
  }

  /**
   * Calculates the linear velocity of the robot
   * @return linear velocity in meters per second
   */
  double getVelocity() {
    return (left_velocity_ + right_velocity_) / 2.0;
  }

  /**
   * Calculates the angular velocity of the robot
   * @return angular velocity in radians per second
   */
  double getAngularVelocity() {
    return (right_velocity_ - left_velocity_) / (TRACK_WIDTH / 2.0);
  }

  /**
   * Sets the left control variable to a given power
   * @param power percent output (-1.0 to 1.0)
   */
  void setLeftAppliedOutput(double power) {
    left_applied_percent_ = power;
  }

  /**
   * Sets the right control variable to a given power
   * @param power percent output (-1.0 to 1.0)
   */
  void setRightAppliedOutput(double power) {
    right_applied_percent_ = power;
  }

  /**
   * Preforms arcade drive on robot
   * @param forward percent of forward power (-1.0 to 1.0)
   * @param turn percent of turn power (-1.0 to 1.0)
   */
  void arcadeDrive(double forward, double turn) {
    setLeftAppliedOutput(forward - turn);
    setRightAppliedOutput(forward + turn);
  }
}
