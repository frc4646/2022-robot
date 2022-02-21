package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class ShooterAim extends CommandBase {
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final Vision vision = RobotContainer.VISION;
  private final OperatorControls controls = RobotContainer.CONTROLS.operator;

  public ShooterAim() {
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    double setpoint = Constants.Shooter.RPM_DEFAULT;

    if (vision.isTargetPresent() && !controls.getFn()) {
      setpoint = vision.getShooterRPM();
    }
    shooter.setClosedLoop(setpoint);
  }
}
