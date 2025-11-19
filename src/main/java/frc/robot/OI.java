package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public abstract class OI {
  // Driver controller object
  static XboxController driver_controller_ = new XboxController(0);

  public static double getDriverLeftYAxis() {
    return -driver_controller_.getLeftY();
  }

  public static double getDriverRightXAxis() {
    return -driver_controller_.getRightX();
  }
    
}
