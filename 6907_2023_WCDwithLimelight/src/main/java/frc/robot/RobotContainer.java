// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveWithJoystickCmd;
import frc.robot.commands.DriveWithLimelightCmd;
import frc.robot.commands.ElevatorWithJoystickCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_DrivetrainSubsystem = new DrivetrainSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
  private final XboxController m_Controller = new XboxController(Constants.ControllerConstants.driverController);

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
        ()->-m_Controller.getLeftY(),
        ()->m_Controller.getRightX()*0.75));

    m_ElevatorSubsystem.setDefaultCommand(
      new ElevatorWithJoystickCmd(m_ElevatorSubsystem,
      ()->m_Controller.getBButton()?1.15:0.05));
  }

  private void configureBindings() {
    new Trigger(m_Controller::getAButton)
        .whileTrue(new RepeatCommand(new DriveWithLimelightCmd(m_DrivetrainSubsystem, m_LimelightSubsystem)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
