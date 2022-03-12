package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class Hood extends SmartSubsystem {
  public static class DataCache {
  }

  private final Servo servo;
  private final DataCache cache = new DataCache();
  public double servoMoveTime;

  private boolean extended = false;

  public Hood() {
    servo = new Servo(Constants.DIGITAL.HOOD);
  }

  @Override
  public void cacheSensors() {

  }

  @Override
  public void updateDashboard(boolean showDetails) {

  }

  public void setOpenLoop(double percent) {
    // motor.set(percent);
  }

  public boolean isOnTarget() {
    return false;  // TODO
  }

  @Override
  public void runTests() {

  }
}
