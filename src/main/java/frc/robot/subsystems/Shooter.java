package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  // TODO motor(s)

  public Shooter() {
    // TODO configure motors
    // TODO limit supply current
    // TODO PIDF
    // TODO peak output forward direction only
    // TODO coast mode
  }

  public void setOpenLoop(double percent) {
    // TODO set percent voltage mode
  }

  public void setSpeed(int velocity) {
    // TODO set velocity control mode
  }

  public int getSpeed() {
    return 0;  // TODO read sensor
  }

  public boolean isOnTarget() {
    // TODO is close enough
    // TODO increment how many times is stable    
    return false;  // TODO decide is enough times is stable
  }
}
