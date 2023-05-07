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
        // Hardware
        public static final int leftJoystickYAxis = 1;
        public static final int rightJoystickYAxis = 5;

        public static final int leftJoystickXAxis = 0;
        public static final int rightJoystickXAxis = 4;

        public static final int leftTriggerAxis = 2;
        public static final int rightTriggerAxis = 3;

        public static final int leftBumperButton = 5;
        public static final int rightBumperButton = 6;

        public static final int aButton = 1;
        public static final int bButton = 2;
        public static final int xButton = 3;
        public static final int yButton = 4;

        public static final int startButton = 8;
        public static final int selectButton = 7;

        public static final int leftJoystickButton = 9;
        public static final int rightJoystickButton = 10;

        public static final int dpadUp = 0;
        public static final int dpadRight = 90;
        public static final int dpadDown = 180;
        public static final int dpadLeft = 270;

        public static final int dpadUpRight = 45;
        public static final int dpadDownRight = 135;
        public static final int dpadDownLeft = 225;
        public static final int dpadUpLeft = 315;

        public static final int dpadNone = -1;

        // Custom
        public static final int driverController = 0;
        public static final int operatorController = 1;

        public static final int autoButton = aButton;

    }

    public static final class LimelightConstants {
        public static final double STEER_K = 0.03;
        public static final double DRIVE_K = 0.26;
        public static final double DESIRED_TARGET_AREA = 13.0;
        public static final double MAX_DRIVE = 0.7;
        
    }
}
