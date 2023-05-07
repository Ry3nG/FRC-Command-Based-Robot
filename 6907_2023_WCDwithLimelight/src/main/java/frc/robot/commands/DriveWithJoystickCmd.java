package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;

public class DriveWithJoystickCmd extends CommandBase {
    private final DrivetrainSubsystem m_DrivetrainSubsystem;
    private final XboxController m_Controller;


    public DriveWithJoystickCmd(DrivetrainSubsystem drivetrainSubsystem, XboxController controller) {
        m_DrivetrainSubsystem = drivetrainSubsystem;
        m_Controller = controller;
        addRequirements(m_DrivetrainSubsystem);
    }

    @Override
    public void execute() {
        double drive = m_Controller.getRawAxis(Constants.ControllerConstants.leftJoystickYAxis);
        double steer = -m_Controller.getRawAxis(Constants.ControllerConstants.rightJoystickXAxis);

        double left = drive + steer;
        double right = drive - steer;

        m_DrivetrainSubsystem.setMotorOutput(left, right);
    }

}
