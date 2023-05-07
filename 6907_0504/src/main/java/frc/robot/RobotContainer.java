// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.OIConstants;

public class RobotContainer {

  // drivetrain
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  // joystick
  private final XboxController joystick1 = new XboxController(OIConstants.kDriverJoystickPort);

  public RobotContainer() {
    configureBindings();

    // DriveSubsystem
    driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(driveSubsystem, //
        () -> joystick1.getRightX(),
        () -> -joystick1.getLeftY())
    );
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
