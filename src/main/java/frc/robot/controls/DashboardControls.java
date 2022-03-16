package frc.robot.controls;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.sequence.FireCargo;
import frc.robot.commands.shooter.ShooterPairTune;
import frc.robot.commands.shooter.ShooterTune;
import frc.robot.commands.turret.TurretOpenLoop;
import frc.robot.commands.turret.TurretPosition;

public class DashboardControls {
  public void configureButtons() {
    SmartDashboard.putData("FireCargo", new FireCargo());
    SmartDashboard.putData("Tune: ShooterPair", new ShooterPairTune());
    SmartDashboard.putData("Tune: Shoot", new ShooterTune());
    
    if (Constants.DRIVETRAIN.TUNING) {
      SmartDashboard.putData("Reset Odometry", new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));}));
    }

    if (Constants.TURRET.TUNING) {
      SmartDashboard.putData("Tune: Turret A", new TurretPosition(Constants.TURRET.SERVO.kHomePosition + 70.0, 0.1));
      SmartDashboard.putData("Tune: Turret Front", new TurretPosition(Constants.TURRET.SERVO.kHomePosition - 180, 0.1));
      SmartDashboard.putData("Tune: Turret Back", new TurretPosition(Constants.TURRET.SERVO.kHomePosition, 0.1).withTimeout(2.0).andThen(new TurretOpenLoop(0.0)));
      SmartDashboard.putData("Tune: Turret Zero", new InstantCommand(RobotContainer.TURRET::zeroSensors));
    }
  }

  public void addLayouts() {
    // addLayout("Commands", RobotContainer.SHOOTER.getName(), RobotContainer.SHOOTER, new ShooterTune(), new ShooterOpenLoop(.5));
    // addLayout("Commands", RobotContainer.INTAKE.getName(), RobotContainer.INTAKE, new IntakeExtend(true));
    // addLayout("Commands", RobotContainer.FEEDER.getName(), RobotContainer.FEEDER, new FeederOpenLoop(.5));
    // addLayout("Commands", "Sequence", new ShootOpenLoop(), new DeployIntake(), new LoadCargo(), new StowIntake());
  }

  /**
   * Add a List layout to a tab
   * @param tab
   * @param name
   * @return
   */
  public static ShuffleboardLayout addLayout(String tab, String name) {
    return addLayout(tab, name);
  }

  /**
   * Add a List of cmd/subsystem/sendable items to a tab
   * @param tab
   * @param name
   * @param items
   * @return
   */
  public static ShuffleboardLayout addLayout(String tab, String name, Sendable... items) {
    ShuffleboardLayout layout = Shuffleboard.getTab(tab)
      .getLayout(name, BuiltInLayouts.kList)
      .withSize(2, 6)
      .withProperties(Map.of("Label position", "HIDDEN"));
    for (Sendable item : items) {
      layout.add(item);
    }
    return layout;
  }

  public static SimpleWidget getGraph(ShuffleboardTab tab, String name, Object defaultValue) {
    return tab.add(name, defaultValue).withWidget(BuiltInWidgets.kGraph);
  }
}
