package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Feeder;

public class FeederAuto extends CommandBase {
  private enum STATE {IDLE, LOADING, LOADED};
  private Feeder feeder = RobotContainer.FEEDER;
  private ColorSensor sensorLoading = RobotContainer.COLOR_SENSOR;
  private STATE stateCurrent = STATE.IDLE;
  private double timeStartState = 0.0;

  public FeederAuto() {
    addRequirements(feeder);
  }

  @Override
  public void initialize() {
    setState(STATE.IDLE);
  }

  @Override
  public void execute() {
    cacheSensors();
    handleState();
  }

  protected void cacheSensors() {
    if (feeder.isShooterLoaded()) {
      setState(STATE.LOADED);
    } else if (sensorLoading.isCorrectCargo()) {
      setState(STATE.LOADING);
    }
  }

  protected void handleState() {
    double setpoint = 0.0;

    if(stateCurrent == STATE.LOADING) {
      setpoint = Constants.Feeder.OPEN_LOOP_LOAD;
      if (Timer.getFPGATimestamp() - timeStartState > Constants.Feeder.TIMEOUT_LOAD) {
        setState(STATE.IDLE);
      }
    }
    feeder.setOpenLoop(setpoint);
    SmartDashboard.putString("Feeder: State", stateCurrent.toString());  // TODO refactor
  }

  private void setState(STATE stateWanted) {
    if (stateWanted != stateCurrent) {
      timeStartState = Timer.getFPGATimestamp();
      stateCurrent = stateWanted;
    }
  }
}
