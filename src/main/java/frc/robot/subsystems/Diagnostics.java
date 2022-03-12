package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.util.DiagnosticState;
import frc.robot.util.LEDColor;

public class Diagnostics extends SmartSubsystem {
  public static final LEDColor
    OFF = new LEDColor(0, 0, 0),
    RED = new LEDColor(255, 0, 0),
    BLUE = new LEDColor(0, 0, 255);

  private final Canifier canifier;
  private final OperatorControls operator = RobotContainer.CONTROLS.getOperator();
  private LEDColor modeDefault = OFF, robotState = OFF;
  private boolean isCriticalIssuePresent = false;
  
  public Diagnostics() {
    canifier = RobotContainer.CANIFIER;
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    canifier.setLEDs(robotState);
    double rumble = (DriverStation.isDisabled() && isCriticalIssuePresent) ? Constants.DIAGNOSTICS.RUMBLE_PERCENT : 0.0;
    operator.setRumble(true, rumble);  // tune which side is better
    operator.setRumble(false, rumble);
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    if (isAutonomous) {
      modeDefault = DriverStation.getAlliance() == Alliance.Red ? RED : BLUE;
    } else {
      modeDefault = OFF;  // Keep drive team sensitive to diagnostics
    }
  }

  @Override
  public void onDisable() {
    modeDefault = DriverStation.getAlliance() == Alliance.Red ? RED : BLUE;
  }

  public void setState(DiagnosticState state) {
    robotState = state.color;
    isCriticalIssuePresent = state.critical;
  }

  public void setStateOkay() {
    robotState = modeDefault;
    isCriticalIssuePresent = false;
  }

  @Override
  public void runTests() {
    // TODO rumble test?
  }
}
