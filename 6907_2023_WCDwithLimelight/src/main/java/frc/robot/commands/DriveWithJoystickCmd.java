package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveWithJoystickCmd extends CommandBase {
    //private final DrivetrainSubsystem m_DrivetrainSubsystem;
    private final DifferentialDrivetrainSubsystem m_DrivetrainSubsystem;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;

    public DriveWithJoystickCmd(DifferentialDrivetrainSubsystem drivetrainSubsystem, DoubleSupplier forward,
            DoubleSupplier rotation) {
        m_DrivetrainSubsystem = drivetrainSubsystem;
        m_forward = forward;
        m_rotation = rotation;
        addRequirements(m_DrivetrainSubsystem);
    }

    @Override
    public void execute() {
        double forward = m_forward.getAsDouble();
        double rotation = m_rotation.getAsDouble();
        
        // implement deadband here instead
        if (Math.abs(forward)<0.1){
            forward = 0;
        }

        if(Math.abs(rotation)<0.1){
            rotation = 0;
        }

        m_DrivetrainSubsystem.arcadeDrive(forward,rotation); 
    }

}
