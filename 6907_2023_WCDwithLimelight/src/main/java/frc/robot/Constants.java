package frc.robot;

public class Constants {
    public static final class DrivetrainConstants {
        public static final int driveMotorLeftMaster = 13;
        public static final int driveMotorLeftSlave = 14;
        public static final int driveMotorLeftVictor = 15;

        public static final int driveMotorRightMaster = 8;
        public static final int driveMotorRightSlave = 9;
        public static final int driveMotorRightVictor = 2;
    }

    public static final class ControllerConstants {
        public static final int driverController = 0;
        public static final int operatorController = 1;

    }

    public static final class LimelightConstants {
        public static final double STEER_K = 0.03;
        public static final double DRIVE_K = 0.26;
        public static final double DESIRED_TARGET_AREA = 13.0;
        public static final double MAX_DRIVE = 0.7;
        public static final double TARGET_VALIDITY_THRESHOLD = 1.0;
        
    }
}
