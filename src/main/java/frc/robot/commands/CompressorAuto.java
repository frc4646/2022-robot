package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;

public class CompressorAuto extends CommandBase {
  private final Pneumatics subsystem = RobotContainer.PNEUMATICS;
  private final Shooter shooter = RobotContainer.SHOOTER;
  private boolean isTeleop = true;

  public CompressorAuto() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    isTeleop = DriverStation.isTeleop();
  }

  @Override
  public void execute() {
    subsystem.setCompressor(isTeleop && !shooter.isShooting());
  }
}
