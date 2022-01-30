package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.sequence.ShootOpenLoop;

public class OperatorControls {    
  private final XboxController operator;
  
  public OperatorControls() {
    operator = new XboxController(2);
    new JoystickButton(operator, Button.kB.value).whenPressed(new IntakeActivate(Constants.Intake.PERCENT_OPEN_LOOP));
    new JoystickButton(operator, Button.kB.value).whenReleased(new IntakeActivate(0.0));

    new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(new ShootOpenLoop(.5));
    new JoystickButton(operator, Button.kLeftBumper.value).whenReleased(new ShootOpenLoop(0.0));
  }
}
