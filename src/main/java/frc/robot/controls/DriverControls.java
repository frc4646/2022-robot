package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.StowIntake;

public class DriverControls {
  private final Joystick throttle, turn;

  DriverControls() {
    throttle = new Joystick(0);
    turn = new Joystick(1);
  }
  
  public void configureButtons() {
    JoystickButton button = new JoystickButton(throttle, 1);
    button.whenPressed(new DeployIntake());
    button.whenReleased(new StowIntake());
  }

  public double getThrottle() { return -throttle.getRawAxis(1); }
  public double getTurning() { return turn.getRawAxis(0); }
  public boolean getQuickturn() { return turn.getRawButton(1); }  
}
