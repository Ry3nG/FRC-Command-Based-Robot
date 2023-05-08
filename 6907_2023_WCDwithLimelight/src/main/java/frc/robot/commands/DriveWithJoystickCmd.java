package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DriveWithJoystickCmd extends CommandBase {
    private final DrivetrainSubsystem m_DrivetrainSubsystem;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;

    public DriveWithJoystickCmd(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier forward, DoubleSupplier rotation) {
        m_DrivetrainSubsystem = drivetrainSubsystem;
        m_forward = forward;
        m_rotation = rotation;
        addRequirements(m_DrivetrainSubsystem);
    }

    @Override
    public void execute() {
        m_DrivetrainSubsystem.arcadeDrive(m_forward.getAsDouble(), m_rotation.getAsDouble()); 
    }

}
