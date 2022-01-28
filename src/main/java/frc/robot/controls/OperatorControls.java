package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.indexer.IndexOpenLoop;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.sequence.ShootOpenLoop;
import frc.robot.commands.shooter.ShooterOpenLoop;
// import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.shooter.ShooterTune;

public class OperatorControls {    
  private final XboxController operator;
  
  public OperatorControls() {
    operator = new XboxController(2);
    new JoystickButton(operator, Button.kB.value).whenPressed(new IntakeActivate(.75));
    new JoystickButton(operator, Button.kB.value).whenReleased(new IntakeActivate(0.0));
  
    // TODO make this a toggle
   

    new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(new ShootOpenLoop());
   // new JoystickButton(operator, Button.kLeftBumper.value).whenReleased(new ShootOpenLoop());

    // TODO add toggle for intake extension
  }
}
