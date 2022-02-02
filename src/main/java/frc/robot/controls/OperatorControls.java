package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.sequence.IndexBall;
import frc.robot.commands.sequence.ShootOpenLoop;
import frc.robot.commands.sequence.StopShoot;

public class OperatorControls {    
  private final XboxController operator;
  
  public OperatorControls() {
    operator = new XboxController(2);
    new JoystickButton(operator, Button.kB.value).whenPressed(new IntakeActivate(Constants.Intake.PERCENT_OPEN_LOOP));
    new JoystickButton(operator, Button.kB.value).whenReleased(new IntakeActivate(0.0));

    new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(new ShootOpenLoop(.5));
    new JoystickButton(operator, Button.kLeftBumper.value).whenReleased(new StopShoot());

    new JoystickButton(operator, Button.kA.value).whenPressed(new IntakeExtend(true));
    new JoystickButton(operator, Button.kA.value).whenReleased(new IntakeExtend(false));
  
    new JoystickButton(operator, Button.kX.value).whenPressed(new IndexBall());
    new JoystickButton(operator, Button.kX.value).whenReleased(new ParallelCommandGroup(new AgitateOpenLoop(0.0),new FeederOpenLoop(0.0)));
  }
}
