package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  // private final NetworkTable table;

  public Vision() {

  }

  public int getShooterSpeed() {
    return 0;  // TODO ideal speed for shooter
  }

  public boolean isTargetPresent() {
    return false;  // TODO
  }
}
