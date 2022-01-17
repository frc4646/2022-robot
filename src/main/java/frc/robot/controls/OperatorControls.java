package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;

public class OperatorControls {    
  private final XboxController operator;
  
  public OperatorControls() {
      operator = new XboxController(2);
  }
}
