package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;

public class DriverControls {
  private final Joystick throttle;
  private final Joystick turn;

  DriverControls() {
    throttle = new Joystick(0);
    turn = new Joystick(1);
  }

  public double getThrottle() {
    return throttle.getRawAxis(1);
  }

  public double getTurning() {
    return -turn.getRawAxis(0);
  }

  public boolean getQuickturn() {
    return turn.getRawButton(1);
  }
}
