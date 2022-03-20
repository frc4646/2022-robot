package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ExhaustIntake;
import frc.robot.commands.sequence.StowIntake;

public class DriverControls {
  private final Joystick throttle, turn;
  private final JoystickButton intake, exhaust;

  DriverControls() {
    throttle = new Joystick(0);
    turn = new Joystick(1);
    intake = new JoystickButton(throttle, 1);
    exhaust = new JoystickButton(throttle, 2);
  }
  
  public void configureButtons() {
    intake.whenPressed(new DeployIntake());
    intake.whenReleased(new StowIntake());
    exhaust.whenActive(new ExhaustIntake());
    exhaust.whenInactive(new StowIntake());
  }

  public double getThrottle() { return -throttle.getRawAxis(1); }
  public double getTurning() { return turn.getRawAxis(0); }
  public boolean getQuickturn() { return turn.getRawButton(1); }  
}
