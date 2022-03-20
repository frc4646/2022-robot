package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.team4646.StabilityCounter;

public class WaitForLevelPitch extends CommandBase {
  private final Drivetrain drive = RobotContainer.DRIVETRAIN;
  private final double pitchAllowed;
  private final int stableCountsWanted;
  private StabilityCounter stability;

  public WaitForLevelPitch(double pitchAllowed, int countsMustBeStable) {
    this.pitchAllowed = pitchAllowed;
    this.stableCountsWanted = countsMustBeStable;
  }

  @Override
  public void initialize() {
    stability = new StabilityCounter(stableCountsWanted);
  }

  @Override
  public void execute() {
    stability.calculate(Math.abs(drive.getPitch().getDegrees()) < pitchAllowed);
  }
  
  @Override
  public boolean isFinished() {
    return stability.isStable();
  }
}
