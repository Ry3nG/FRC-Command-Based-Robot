package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveWithLimelight extends CommandBase {
    private final DrivetrainSubsystem m_DrivetrainSubsystem;
    private final LimelightSubsystem m_LimelightSubsystem;

    public DriveWithLimelight(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem) {
        m_DrivetrainSubsystem = drivetrainSubsystem;
        m_LimelightSubsystem = limelightSubsystem;
        addRequirements(m_DrivetrainSubsystem, m_LimelightSubsystem);
    }

    @Override
    public void execute() {
        double rotation = m_LimelightSubsystem.getTargetX()*Constants.LimelightConstants.STEER_K;
        double forward = (Constants.LimelightConstants.DESIRED_TARGET_AREA - m_LimelightSubsystem.getTargetArea()) * Constants.LimelightConstants.DRIVE_K;

        if(forward>Constants.LimelightConstants.MAX_DRIVE) {
            forward = Constants.LimelightConstants.MAX_DRIVE;
        }

        if(m_LimelightSubsystem.getTargetValidity() >= 1.0) {
            m_DrivetrainSubsystem.arcadeDrive(forward, rotation);
        } else {
            m_DrivetrainSubsystem.arcadeDrive(0, 0);
        }
    }
}
