package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.indexer.IndexOpenLoop;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.shooter.ShooterVelocity;

public class OperatorControls {    
  private final XboxController operator;
  
  public OperatorControls() {
    operator = new XboxController(2);
    new JoystickButton(operator, Button.kB.value).whenPressed(new IntakeActivate(1.0));
    new JoystickButton(operator, Button.kB.value).whenReleased(new IntakeActivate(0.0));
  
    // TODO make this a toggle
    new JoystickButton(operator, Button.kRightBumper.value).whenPressed(new ShooterVelocity(1.0));
    new JoystickButton(operator, Button.kRightBumper.value).whenReleased(new ShooterVelocity(0.0));

    new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(new IndexOpenLoop(.5));
    new JoystickButton(operator, Button.kLeftBumper.value).whenReleased(new IndexOpenLoop(0.0));

    // TODO add toggle for intake extension
  }
}
