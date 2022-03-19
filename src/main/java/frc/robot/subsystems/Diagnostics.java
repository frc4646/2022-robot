package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.util.DiagnosticState;
import frc.team4646.LEDColor;

public class Diagnostics extends SmartSubsystem {
  public final LEDColor
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
    setLEDs();
    setRumble((DriverStation.isDisabled() && isCriticalIssuePresent) ? Constants.DIAGNOSTICS.RUMBLE_PERCENT : 0.0);
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    modeDefault = !isAutonomous ? OFF : allianceColor();
  }

  @Override
  public void onDisable() {
    modeDefault = allianceColor();
  }

  public void setState(DiagnosticState state) {
    robotState = state.color;
    isCriticalIssuePresent = state.critical;
  }

  public void setStateOkay() {
    robotState = modeDefault;
    isCriticalIssuePresent = false;
  }

  private void setLEDs() {
    canifier.setLEDs(robotState);
  }

  private void setRumble(double percent) {
    operator.setRumble(true, percent);
    operator.setRumble(false, percent);
  }

  private LEDColor allianceColor() { return DriverStation.getAlliance() == Alliance.Red ? RED : BLUE; }
}
