package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

public class Climber extends SmartSubsystem {
  // TODO motors

  private boolean isBrakeMode;

  public Climber() {
    // TODO configure motors

    isBrakeMode = false;
    setBrakeMode(true);
  }

  public void setBrakeMode(boolean enable) {
    if (isBrakeMode != enable) {
      IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
      // TODO set mode on motors
      isBrakeMode = enable;
    }
  }

  public void setOpenLoop(double percent) {
    // TODO
  }
}
