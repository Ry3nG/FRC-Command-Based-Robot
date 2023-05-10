package frc.robot.subsystems;

import javax.print.DocFlavor;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DifferentialDrivetrainSubsystem extends SubsystemBase {
    private final CANSparkMax m_driveMotorLeftMaster = new CANSparkMax(Constants.DrivetrainConstants.driveMotorLeftMaster, MotorType.kBrushless);
    private final CANSparkMax m_driveMotorLeftSlave = new CANSparkMax(Constants.DrivetrainConstants.driveMotorLeftSlave, MotorType.kBrushless);
    private final Victor m_driveMotorLeftVictor = new Victor(Constants.DrivetrainConstants.driveMotorLeftVictor);

    private final CANSparkMax m_driveMotorRightMaster = new CANSparkMax(Constants.DrivetrainConstants.driveMotorRightMaster, MotorType.kBrushless);
    private final CANSparkMax m_driveMotorRightSlave = new CANSparkMax(Constants.DrivetrainConstants.driveMotorRightSlave, MotorType.kBrushless);
    private final Victor m_driveMotorRightVictor = new Victor(Constants.DrivetrainConstants.driveMotorRightVictor);


    private final DifferentialDrive m_drive = new DifferentialDrive(m_driveMotorLeftMaster,m_driveMotorRightMaster);
    private final DifferentialDrive m_driveVictor = new DifferentialDrive(m_driveMotorLeftVictor, m_driveMotorRightVictor);
    
    // encoders
    private final RelativeEncoder m_leftEncoder = m_driveMotorLeftMaster.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_driveMotorRightMaster.getEncoder();

    // gyro sensor, we used pigeon1
    private final TalonSRX m_gyro_srx = new TalonSRX(6);
    private final PigeonIMU m_gyro = new PigeonIMU(m_gyro_srx);
    private double m_lastYaw = 0;
    private double m_lastTimestamp = Timer.getFPGATimestamp();

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    /* Constructor for the Drivetrain subsystem */
    public DifferentialDrivetrainSubsystem() {
        m_driveMotorLeftSlave.follow(m_driveMotorLeftMaster);
        m_driveMotorRightSlave.follow(m_driveMotorRightMaster);

        // // invert right side motors
        m_driveMotorRightMaster.setInverted(true);
        m_driveMotorRightSlave.setInverted(true);
        m_driveMotorRightVictor.setInverted(true);
        

        // set the distance per pulse for the encoders here
        m_leftEncoder.setPositionConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setPositionConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse);

        resetEncoders();
        m_gyro.configFactoryDefault();

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getPigeonYaw()), m_leftEncoder.getPosition(),
                m_rightEncoder.getPosition());
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        Rotation2d heading = Rotation2d.fromDegrees(getPigeonYaw());
        SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());
        SmartDashboard.putString("Pigeon State", m_gyro.getState().toString());
        SmartDashboard.putNumber("Pigeon Heading Degree", heading.getDegrees());
        m_odometry.update(heading, m_leftEncoder.getPosition(),
                m_rightEncoder.getPosition());
    
    }

    /**
     * Returns the currently-estimated pose of the robot.
     * 
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     * 
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     * 
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(Rotation2d.fromDegrees(getPigeonYaw()), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
    }
    
    /**
     * Controls the left and right side of the drive directly with voltages.
     * 
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {

        SmartDashboard.putNumber("Left Volt", leftVolts);
        SmartDashboard.putNumber("Right Volts", rightVolts);
        m_driveMotorLeftMaster.setVoltage(leftVolts);
        m_driveMotorLeftSlave.setVoltage(leftVolts);
        m_driveMotorLeftVictor.setVoltage(leftVolts);

        m_driveMotorRightMaster.setVoltage(rightVolts);
        m_driveMotorRightSlave.setVoltage(rightVolts);
        m_driveMotorRightVictor.setVoltage(rightVolts);
        m_drive.feed();
        m_driveVictor.feed();
    }


    /**
     * Resets the drive encoders to currently read a position of 0.
     */

    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    /**
     * Drives the robot using arcade controls.
     * 
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
        m_driveVictor.arcadeDrive(fwd, rot);
    }

    /**
     * Gets the average distance of the two encoders.
     * 
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     * 
     * @return the left drive encoder
     */
    public RelativeEncoder getLeftEncoder() {
        return m_leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     * 
     * @return the right drive encoder
     */
    public RelativeEncoder getRightEncoder() {
        return m_rightEncoder;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     * 
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
        m_driveVictor.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        m_gyro.setFusedHeading(0.0);
    }

    /**
     * Returns the heading of the robot.
     * 
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getFusedHeading();
    }

    /**
     * Returns the turn rate of the robot.
     * 
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        double currentYaw = getPigeonYaw();
        double currentTimestamp = Timer.getFPGATimestamp();
        
        double yawRate = (currentYaw - m_lastYaw) / (currentTimestamp - m_lastTimestamp);
        
        m_lastYaw = currentYaw;
        m_lastTimestamp = currentTimestamp;
        
        return yawRate;
    }
    

    private double getPigeonYaw() {
        return m_gyro.getFusedHeading();
        // double[] ypr = new double[3];
        // m_gyro.getYawPitchRoll(ypr);
        // return ypr[0];
    }

    }

