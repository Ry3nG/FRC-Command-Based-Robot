package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX mElevator = new TalonFX(Constants.ElevatorConstants.elevatorMotor);
    public final double MAX_HEIGHT = 1.20;
    public final double MIN_HEIGHT = 0.05;
 
    //MAX_ENCODER: 27000
    public final double ENCODER_TICKS_PER_METER = 223000;

    private final double MAXV_TICKS_PER_100MS = (MAX_HEIGHT-MIN_HEIGHT)*ENCODER_TICKS_PER_METER/1.0/10.0;
    private final double MAXA_TICKS_PER_100MS_PER_SEC = MAXV_TICKS_PER_100MS*2;

    /*
     * ElevatorSubsystem() is the constructor for the DrivetrainSubsystem class
     * 
     * This method initializes the elevator motor and set the encoder reading to 0
     */
    public ElevatorSubsystem() {
        mElevator.configFactoryDefault();
        mElevator.setSelectedSensorPosition(0);
        //TODO: invert or not
        mElevator.setInverted(false);
        
        //TODO: maximum height
        mElevator.configReverseSoftLimitEnable(true);
        mElevator.configReverseSoftLimitThreshold(0);
        mElevator.configForwardSoftLimitEnable(true);
        mElevator.configForwardSoftLimitThreshold(ENCODER_TICKS_PER_METER * MAX_HEIGHT);

        //Config motion magic parameters
        mElevator.configMotionCruiseVelocity((MAX_HEIGHT-MIN_HEIGHT)*ENCODER_TICKS_PER_METER/1.0/10.0);
        mElevator.configMotionAcceleration(MAXA_TICKS_PER_100MS_PER_SEC);
        
        //Config pidf
        mElevator.config_kF(0, 0.045,10);
        mElevator.config_kP(0, 0.02, 10);
        mElevator.config_kI(0, 0.0, 10);
        mElevator.config_kD(0, 0.0, 10);
    }

    /**
     * set elevator height relative to initial position
     * @param height_meters the target height of the elevator relative to the starting position
     * 
     */
    public void setHeight(double height_meters){
        height_meters = Math.min(MAX_HEIGHT, Math.max(MIN_HEIGHT, height_meters));
        mElevator.set(ControlMode.MotionMagic, height_meters*ENCODER_TICKS_PER_METER);
    }

    public double getElevatorEncoder(){
        return mElevator.getSelectedSensorPosition();
    }
    
}
