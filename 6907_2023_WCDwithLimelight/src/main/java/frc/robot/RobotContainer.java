// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveWithJoystickCmd;
import frc.robot.commands.DriveWithLimelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_DrivetrainSubsystem = new DrivetrainSubsystem();
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
    m_DrivetrainSubsystem.setDefaultCommand(new DriveWithJoystickCmd(m_DrivetrainSubsystem, m_Controller));
       
    
  }

  private void configureBindings() {
    JoystickButton autoButton = new JoystickButton(m_Controller, Constants.ControllerConstants.autoButton);
    autoButton.whileTrue(new DriveWithLimelight(m_DrivetrainSubsystem, m_LimelightSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
