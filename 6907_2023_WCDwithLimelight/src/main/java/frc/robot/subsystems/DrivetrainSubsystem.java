package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase{
    private final CANSparkMax driveMotorLeftMaster = new CANSparkMax(Constants.DrivetrainConstants.driveMotorLeftMaster, MotorType.kBrushless);
    private final CANSparkMax driveMotorLeftSlave = new CANSparkMax(Constants.DrivetrainConstants.driveMotorLeftSlave, MotorType.kBrushless);
    private final Victor driveMotorLeftVictor = new Victor(Constants.DrivetrainConstants.driveMotorLeftVictor);

    private final CANSparkMax driveMotorRightMaster = new CANSparkMax(Constants.DrivetrainConstants.driveMotorRightMaster, MotorType.kBrushless);
    private final CANSparkMax driveMotorRightSlave = new CANSparkMax(Constants.DrivetrainConstants.driveMotorRightSlave, MotorType.kBrushless);
    private final Victor driveMotorRightVictor = new Victor(Constants.DrivetrainConstants.driveMotorRightVictor);

    /*
     * DrivetrainSubsystem() is the constructor for the DrivetrainSubsystem class
     * 
     * This constructor sets the right slave motor to follow the right master motor, and the left slave motor to follow the left master motor
     * be mind that the victors cannot directly follow the sparkmaxes, so we have to set the output manually
     */
    public DrivetrainSubsystem() {
        driveMotorLeftSlave.follow(driveMotorLeftMaster);
        driveMotorRightSlave.follow(driveMotorRightMaster);
    }

    /*
     * setMotorOutput() sets the output of the drivetrain motors
     * 
     * @param left the output of the left motors
     * @param right the output of the right motors
     * 
     * This method sets the output of the sparkmaxes and the victors to the same value
     */
    public void setMotorOutput(double left, double right) {
        driveMotorLeftMaster.set(left);
        driveMotorRightMaster.set(right);

        // manually set the victors to the same output as the sparkmaxes
        driveMotorLeftVictor.set(left);
        driveMotorRightVictor.set(right);
    }
}
