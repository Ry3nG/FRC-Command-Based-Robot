// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveWithJoystickCmd;
import frc.robot.commands.DriveWithLimelightCmd;
import frc.robot.subsystems.DifferentialDrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  //private final DrivetrainSubsystem m_DrivetrainSubsystem = new DrivetrainSubsystem();
  private final DifferentialDrivetrainSubsystem m_DrivetrainSubsystem = new DifferentialDrivetrainSubsystem();
  private final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
  private final XboxController m_Controller = new XboxController(Constants.XboxConstants.driverController);

  /*
   * RobotContainer() is the constructor for the RobotContainer class
   * 
   * This constructor sets up the default commands for the subsystems
   */
  public RobotContainer() {
    configureBindings();

    // Drivesubsystem
    m_DrivetrainSubsystem.setDefaultCommand(
        // a split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        // the library is somehow reversed so the left and right are inversed
        new DriveWithJoystickCmd(m_DrivetrainSubsystem,
            () -> -m_Controller.getLeftY(),
            () -> -m_Controller.getRightX() * 0.75));

  }

  private void configureBindings() {
    new Trigger(m_Controller::getAButton)
        .whileTrue(new RepeatCommand(new DriveWithLimelightCmd(m_DrivetrainSubsystem, m_LimelightSubsystem)));
  }

  public Command getAutonomousCommand() {
    
    // create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.ControlConstants.ksVolts,
            Constants.ControlConstants.kvVoltSecondsPerMeter, Constants.ControlConstants.kaVoltSecondsSquaredPerMeter),
        Constants.ControlConstants.kDriveKinematics, 10);

    // create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(Constants.ControlConstants.kMaxSpeedMetersPerSecond,
        Constants.ControlConstants.kMaxAccelerationMetersPerSecondSquared)
        // add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.ControlConstants.kDriveKinematics)
        // apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory examplTrajectory = TrajectoryGenerator.generateTrajectory(
        // start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1),new Translation2d(2,-1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

      RamseteCommand ramseteCommand = new RamseteCommand(
        examplTrajectory,
        m_DrivetrainSubsystem::getPose,
        new RamseteController(Constants.ControlConstants.kRamseteB, Constants.ControlConstants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ControlConstants.ksVolts,
            Constants.ControlConstants.kvVoltSecondsPerMeter, Constants.ControlConstants.kaVoltSecondsSquaredPerMeter),
        Constants.ControlConstants.kDriveKinematics,
        m_DrivetrainSubsystem::getWheelSpeeds,
        new PIDController(Constants.ControlConstants.kPDriveVel, 0, 0),
        new PIDController(Constants.ControlConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_DrivetrainSubsystem::tankDriveVolts,
        m_DrivetrainSubsystem);

      // reset odometry to the starting pose of the trajectory.
      m_DrivetrainSubsystem.resetOdometry(examplTrajectory.getInitialPose());

      // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> m_DrivetrainSubsystem.tankDriveVolts(0, 0));
      
  }
  
}
