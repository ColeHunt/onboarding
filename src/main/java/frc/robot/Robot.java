// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private static final boolean INVERT_LEFT_DRIVE = false;
  TalonSRX fl_drive_motor_;
  TalonSRX fr_drive_motor_;
  TalonSRX bl_drive_motor_;
  TalonSRX br_drive_motor_;

  XboxController driver_controller_;

  private static final double WHEEL_RADIUS = Units.inchesToMeters(3.0); // in inches 
  private static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
  private static final double TRACK_WIDTH = Units.inchesToMeters(24.0); // in inches
  DifferentialDrive drive_train_;
  DifferentialDriveKinematics drive_train_kinematics_;
  Pose2d chassis_pose_ = new Pose2d();
  ChassisSpeeds chassis_speeds_ = new ChassisSpeeds();
  Rotation2d chassis_yaw_ = new Rotation2d(0);
  DifferentialDrivePoseEstimator pose_estimator_;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Initialize motor controllers
    fl_drive_motor_ = new TalonSRX(1);
    bl_drive_motor_ = new TalonSRX(2);
    fr_drive_motor_ = new TalonSRX(3);
    br_drive_motor_ = new TalonSRX(4);

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

    // Initialize drive train object
    drive_train_ = new DifferentialDrive(this::setLeftDrivePower, this::setRightDrivePower);
    drive_train_kinematics_ = new DifferentialDriveKinematics(TRACK_WIDTH);
    pose_estimator_ = new DifferentialDrivePoseEstimator(
      drive_train_kinematics_, 
      chassis_yaw_, 
      0.0, 
      0.0, 
      chassis_pose_);

    // Initialize driver controller
    driver_controller_ = new XboxController(0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    double left_velocity_ = fl_drive_motor_.getSelectedSensorVelocity() / 4096.0  * WHEEL_CIRCUMFERENCE * 10; // m/s
    double right_velocity_ = fr_drive_motor_.getSelectedSensorVelocity() / 4096.0  * WHEEL_CIRCUMFERENCE * 10; // m/s
    double left_position_ = fl_drive_motor_.getSelectedSensorPosition() / 4096.0  * WHEEL_CIRCUMFERENCE; // m
    double right_position_ = fr_drive_motor_.getSelectedSensorPosition() / 4096.0 * WHEEL_CIRCUMFERENCE; // m
    DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(left_velocity_, right_velocity_);

    // Calculate odometry
    chassis_speeds_ = drive_train_kinematics_.toChassisSpeeds(speeds);
    chassis_yaw_ = calcChassisYaw();
    pose_estimator_.update(chassis_yaw_, left_position_, right_position_);
    
    // Display odometry information to dashboard
    SmartDashboard.putNumber("Chassis Distance", chassis_pose_.getX());
    SmartDashboard.putNumber("Chassis Velocity", chassis_speeds_.vxMetersPerSecond);
    SmartDashboard.putNumber("Chassis Yaw", chassis_yaw_.getDegrees());
    SmartDashboard.putNumber("Chassis Yaw Rate", chassis_speeds_.omegaRadiansPerSecond);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    fl_drive_motor_.setSelectedSensorPosition(0);
    fr_drive_motor_.setSelectedSensorPosition(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if(chassis_pose_.getX() < 9.9){
      // Drive Forward
      drive_train_.arcadeDrive(0.25, 0);
    } else if (chassis_pose_.getX() > 10.1){
      // Drive Backward
      drive_train_.arcadeDrive(-0.25, 0);
    } else {
      // Stop
      drive_train_.stopMotor();
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drive_train_.arcadeDrive(-driver_controller_.getLeftY(), driver_controller_.getLeftX());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /**
   * Set left drive power
   * @param power Power from -1.0 to 1.0
   */
  private void setLeftDrivePower(double power) {
    fl_drive_motor_.set(ControlMode.PercentOutput, power);
  }

  /**
   * Set right drive power
   * @param power Power from -1.0 to 1.0
   */
  private void setRightDrivePower(double power) {
    fr_drive_motor_.set(ControlMode.PercentOutput, power);
  }

  /**
   * Get chassis yaw
   * @return Chassis yaw between as Rotation2d
   */
  private Rotation2d calcChassisYaw(){
    return chassis_yaw_.rotateBy(new Rotation2d(chassis_speeds_.toTwist2d(kDefaultPeriod).dtheta));
  }

}
