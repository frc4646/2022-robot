package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.intake.IntakeActivate;

public class OperatorControls {    
  private final XboxController operator;
  
  public OperatorControls() {
    operator = new XboxController(2);
    new JoystickButton(operator, Button.kB.value).whenPressed(new IntakeActivate());
  }
}
