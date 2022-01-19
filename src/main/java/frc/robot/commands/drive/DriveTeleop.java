package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.controls.DriverControls;
import frc.robot.subsystems.DriveSubsystem;
import frc.team254.util.DriveSignal;
import frc.team254.util.OpenLoopCheesyDriveHelper;

public class DriveTeleop extends CommandBase {
  private final DriveSubsystem subsystem = RobotContainer.DRIVE;
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
