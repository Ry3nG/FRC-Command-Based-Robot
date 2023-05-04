package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    private final Spark driveLeftMotor = new Spark(0);
    private final Spark driveRightMotor  = new Spark(1);
    private final Encoder driveLeftEncoder = new Encoder(0, 1);
    private final Encoder driveRightEncoder = new Encoder(2, 3);
    private final double kEncoderTick2Meter = 1.0/4096.0*0.128*Math.PI;

    public double getEncoderMeters(){
        return (driveLeftEncoder.get() + driveRightEncoder.get()) / 2.0 * kEncoderTick2Meter;
    }

    public DriveSubsystem(){

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Drive encoder value: ", getEncoderMeters());
    }


    public void setMotorOutput(double left, double right){
        driveLeftMotor.set(left);
        driveRightMotor.set(right);
    }
    
}
