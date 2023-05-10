package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Constants {
    public static final class DrivetrainConstants {
        public static final int driveMotorLeftMaster = 14;
        public static final int driveMotorLeftSlave = 13;
        public static final int driveMotorLeftVictor = 15;

        public static final int leftEncoderA = 14;
        public static final int leftEncoderB = 13;

        public static final int driveMotorRightMaster = 8;
        public static final int driveMotorRightSlave = 9;
        public static final int driveMotorRightVictor = 2;

        public static final int rightEncoderA = 8;
        public static final int rightEncoderB = 9;

        public static final int gyroPort = 6;

        public static final double kTrackWidthMeters = 0.64621;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kEncoderCountsPerRevolution = 1;
        public static final double kGearRatio = 10.875;
        public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI)
                / (kEncoderCountsPerRevolution * kGearRatio);
    }

    public static final class XboxConstants {
        public static final int driverController = 0;
        public static final int operatorController = 1;

    }

    public static final class LimelightConstants {
        public static final double STEER_K = 0.05;
        public static final double DRIVE_K = 0.13;
        public static final double DESIRED_TARGET_AREA = 3.6;
        public static final double MAX_DRIVE = 0.5;
        public static final double TARGET_VALIDITY_THRESHOLD = 1.0;

    }

    public static final class ControlConstants {
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                Constants.DrivetrainConstants.kTrackWidthMeters);

        public static final double ksVolts = 0.090943;
        public static final double kvVoltSecondsPerMeter = 2.9638;
        public static final double kaVoltSecondsSquaredPerMeter = 0.78201;

        //public static final double kPDriveVel = 3.6877;
        public static final double kPDriveVel = 4.1991E-06;

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
