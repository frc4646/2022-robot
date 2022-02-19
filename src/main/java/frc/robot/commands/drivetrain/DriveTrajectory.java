// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;

public class DriveTrajectory extends CommandGroup {
  /** Add your docs here. */
  public DriveTrajectory(TrajectortyConfig trajectory) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    addSequential(new RamseteCommand(trajectory, 
    pose, //m_robotdirive::getPose
    new RamseteController(Constants.Drivetrain.RAMSETE_B,Constants.Drivetrain.RAMSETE_ZETA), 
    new SimpleMotorFeedforward(
      Constants.Drivetrain.FEED_FORWARD_GAIN_STATIC, 
      Constants.Drivetrain.FEED_FORWARD_GAIN_VELOCITY,
      Constants.Drivetrain.FEED_FORWARD_GAIN_ACCEL), 
    Constants.Drivetrain.DRIVE_KINEMATICS, 
    wheelSpeeds, //m_robotdrive::getWheelSpeeds
    new PIDController(Constants.Drivetrain., ki, kd), 
    rightController, 
    outputVolts, 
    requirements))
    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
