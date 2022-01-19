package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.controls.DriverControls;
import frc.robot.subsystems.Drivetrain;
import frc.team254.util.DriveSignal;
import frc.team254.util.OpenLoopCheesyDriveHelper;

public class DriveTeleop extends CommandBase {
  private final Drivetrain subsystem = RobotContainer.DRIVETRAIN;
  private final DriverControls controls = RobotContainer.CONTROLS.driver;
  private final OpenLoopCheesyDriveHelper steeringController = OpenLoopCheesyDriveHelper.getInstance();;

  public DriveTeleop() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    DriveSignal output = steeringController.cheesyDrive(controls.getThrottle(), controls.getTurning(), controls.getQuickturn());
    subsystem.setOpenLoop(output.getLeft(), output.getRight());
  }
}
