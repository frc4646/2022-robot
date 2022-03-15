package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;

public class ShooterPairTune extends CommandBase {
  public Shooter subsystem1 = RobotContainer.SHOOTER;
  public ShooterTop subsystem2 = RobotContainer.SHOOTER_TOP;

  public ShooterPairTune() {
    addRequirements(subsystem1, subsystem2);
    SmartDashboard.putNumber("ShooterPairTune Bottom", 2400.0);
  }

  @Override
  public void execute() {
    subsystem1.setClosedLoop(SmartDashboard.getNumber("ShooterPairTune Bottom", 0.0));
    subsystem2.setClosedLoop(SmartDashboard.getNumber("ShooterPairTune Bottom", 0.0) * 1.83);
    // subsystem2.setOpenLoop(SmartDashboard.getNumber("ShooterPairTune Top", 0.0));
  }
}
