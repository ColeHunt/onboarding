// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
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

  XboxController controller = new XboxController(0);

  private static final double WHEEL_RADIUS = Units.inchesToMeters(3.0); // in inches 
  private static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
  double chassis_trans_velocity_;
  double chassis_rot_velocity_;
  double chassis_trans_pos_;

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
    double left_velocity = fl_drive_motor_.getSelectedSensorVelocity() / 4096.0  * WHEEL_CIRCUMFERENCE * 10; // m/s
    double right_velocity = fr_drive_motor_.getSelectedSensorVelocity() / 4096.0  * WHEEL_CIRCUMFERENCE * 10; // m/s
    double left_postion = fl_drive_motor_.getSelectedSensorPosition() / 4096.0  * WHEEL_CIRCUMFERENCE; // m
    double right_position = fr_drive_motor_.getSelectedSensorPosition() / 4096.0 * WHEEL_CIRCUMFERENCE; // m

    chassis_trans_pos_ = (left_postion + right_position) / 2.0;
    chassis_trans_velocity_ = (left_velocity + right_velocity) / 2.0;
    chassis_rot_velocity_ = (right_velocity - left_velocity) / 2.0; // rads/s

    SmartDashboard.putNumber("Chassis Velocity", chassis_trans_velocity_);
    SmartDashboard.putNumber("Chassis Distance", chassis_trans_pos_);
    SmartDashboard.putNumber("Chassis Rot Velocity", chassis_rot_velocity_);
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
    if(chassis_trans_pos_ < 9.9){
      // Drive Forward
      fl_drive_motor_.set(ControlMode.PercentOutput, 0.25);
      fr_drive_motor_.set(ControlMode.PercentOutput, 0.25);
    } else if (chassis_trans_pos_ > 10.1){
      // Drive Backward
      fl_drive_motor_.set(ControlMode.PercentOutput, -0.25);
      fr_drive_motor_.set(ControlMode.PercentOutput, -0.25);
    } else {
      // Stop
      fl_drive_motor_.set(ControlMode.PercentOutput, 0.0);
      fr_drive_motor_.set(ControlMode.PercentOutput, 0.0);
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    fl_drive_motor_.set(ControlMode.PercentOutput, -controller.getLeftY());
    fr_drive_motor_.set(ControlMode.PercentOutput, -controller.getRightY());
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
}
