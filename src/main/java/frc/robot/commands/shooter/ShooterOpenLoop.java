package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterOpenLoop extends CommandBase {
    public ShooterSubsystem subsystem = RobotContainer.SHOOTER;

    public final double percent;

    public ShooterOpenLoop(double percent) {
        addRequirements(subsystem);
        this.percent = percent;
    }

    @Override
    public void initialize() {
      subsystem.setOpenLoop(percent);
    }
}
