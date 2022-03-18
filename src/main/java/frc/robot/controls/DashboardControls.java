package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DriveTune;
import frc.robot.commands.sequence.FireCargo;
import frc.robot.commands.shooter.ShooterTune;
import frc.robot.commands.turret.TurretPosition;

public class DashboardControls {
  public void configureButtons() {
    if (Constants.TUNING.DRIVETRAIN) {
      SmartDashboard.putData("Reset Odometry", new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));}));
      SmartDashboard.putData("Drive Velocity", new DriveTune(0.75));
    }
    if (Constants.TUNING.SHOOTERS) {
      SmartDashboard.putData("FireCargo", new FireCargo());
      SmartDashboard.putData("Tune: Shoot", new ShooterTune());
    }
    if (Constants.TUNING.TURRET) {
      SmartDashboard.putData("Tune: Turret A", new TurretPosition(Constants.TURRET.SERVO.kHomePosition + 70.0, 0.1));
      SmartDashboard.putData("Tune: Turret B", new TurretPosition(Constants.TURRET.SERVO.kHomePosition - 70.0, 0.1));
      SmartDashboard.putData("Tune: Turret Zero", new InstantCommand(RobotContainer.TURRET::zeroSensors));
    }
  }
}
