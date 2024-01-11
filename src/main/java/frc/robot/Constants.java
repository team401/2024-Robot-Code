package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.drive.CommandSwerveDrivetrain;

public final class Constants {

    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : Mode.SIM;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final class CANDevices {

        public static final String canivoreName = "Canivore";

        public static final int frontLeftDriveMotorID = 2;
        public static final int frontLeftRotationMotorID = 1;

        public static final int frontRightDriveMotorID = 4;
        public static final int frontRightRotationMotorID = 3;

        public static final int backLeftDriveMotorID = 8;
        public static final int backLeftRotationMotorID = 7;

        public static final int backRightDriveMotorID = 6;
        public static final int backRightRotationMotorID = 5;

        public static final int frontLeftRotationEncoderID = 9;
        public static final int frontRightRotationEncoderID = 10;
        public static final int backLeftRotationEncoderID = 12;
        public static final int backRightRotationEncoderID = 11;

        public static final int pigeonIMU = 20;

        public static final int leftPivotMotorID = 14;
        public static final int rightPivotMotorID = 13;

        public static final int pivotEncoderID = 0;

        public static final int telescopeMotorID = 18;

        public static final int wristMotorID = 17;

        public static final int rightIntakeMotorID = 15;
        public static final int leftIntakeMotorID = 16;

    }

    public static final class DIOPorts {

        public static final int switch1 = 1;
        public static final int switch2 = 2;
    }

    public static final class DriveConstants {

        public static final double MaxSpeedMetPerSec = 6;
        public static final double MaxAngularRateRadiansPerSec = Math.PI * 2; // 2 PI is one full rotation per second
        public static final double deadbandPercent = 0.16;

        public static final double trackWidth = Units.inchesToMeters(17.75); // distance between the left and right
                                                                             // wheels
        public static final double wheelBase = Units.inchesToMeters(23.75); // distance between the front and rear
                                                                            // wheels
        public static final double wheelRadiusM = Units.inchesToMeters(2.02);

        public static final double driveWheelGearReduction = 6.86;
        public static final double rotationWheelGearReduction = 12.8;

        public static final double frontLeftAngleOffset = 2.922;
        public static final double frontRightAngleOffset = -1.80;
        public static final double backLeftAngleOffset = 1.681;
        public static final double backRightAngleOffset = 1.039;

        public static final double[] driveKps = { 0.7, 0.4, 0.7, 0.7 };
        public static final double[] driveKds = { 3.5, 2.5, 3.7, 3.5 };

        public static final double[] rotationKps = { 7, 7, 7, 7 };
        public static final double[] rotationKds = { 0, 0, 0, 0 };

        public static final double driveSnapKp = 1.5;
        public static final double driveSnapKi = 0;
        public static final double driveSnapKd = 0;

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right
        );

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.24, 2.185);

        public static final double maxDriveSpeed = 5;
        public static final double maxTurnRate = 2 * Math.PI;

        public static final double driveJoystickDeadbandPercent = 0.12;
        public static final double driveMaxJerk = 200.0;

        public static final double poseMoveTranslationkP = 1;
        public static final double poseMoveTranslationMaxVel = 3;
        public static final double poseMoveTranslationMaxAccel = 3;

        public static final double poseMoveRotationkP = 0.05;
        public static final double poseMoveRotationMaxVel = Math.PI;
        public static final double poseMoveRotationMaxAccel = Math.PI;

    }

    public static final class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot
        // The steer motor uses MotionMagicVoltage control
        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(7).withKI(0).withKD(0)
                .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses:
        // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
        // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
        private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(0.7).withKI(0).withKD(3.5)
                .withKS(0).withKV(0).withKA(0);

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 300.0;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        private static final double kSpeedAt12VoltsMps = 6.0;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.746031746031747;
        private static final double kSteerGearRatio = 21.428571428571427;
        private static final double kWheelRadiusInches = 2.02;

        private static final boolean kSteerMotorReversed = true;
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final String kCANbusName = "Canivore";
        private static final int kPigeonId = 20;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;

        private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withPigeon2Id(kPigeonId)
                .withCANbusName(kCANbusName);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withWheelRadius(kWheelRadiusInches)
                .withSlipCurrent(kSlipCurrentA)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withCouplingGearRatio(kCoupleRatio)
                .withSteerMotorInverted(kSteerMotorReversed);

        // Front Left
        private static final int kFrontLeftDriveMotorId = 2;
        private static final int kFrontLeftSteerMotorId = 1;
        private static final int kFrontLeftEncoderId = 9;
        private static final double kFrontLeftEncoderOffset = -0.4609375;

        private static final double kFrontLeftXPosInches = 11.875;
        private static final double kFrontLeftYPosInches = 8.875;

        // Front Right
        private static final int kFrontRightDriveMotorId = 4;
        private static final int kFrontRightSteerMotorId = 3;
        private static final int kFrontRightEncoderId = 10;
        private static final double kFrontRightEncoderOffset = -0.70751953125;

        private static final double kFrontRightXPosInches = 11.875;
        private static final double kFrontRightYPosInches = -8.875;

        // Back Left
        private static final int kBackLeftDriveMotorId = 8;
        private static final int kBackLeftSteerMotorId = 7;
        private static final int kBackLeftEncoderId = 12;
        private static final double kBackLeftEncoderOffset = -0.265380859375;

        private static final double kBackLeftXPosInches = -11.875;
        private static final double kBackLeftYPosInches = 8.875;

        // Back Right
        private static final int kBackRightDriveMotorId = 6;
        private static final int kBackRightSteerMotorId = 5;
        private static final int kBackRightEncoderId = 11;
        private static final double kBackRightEncoderOffset = -0.17724609375;

        private static final double kBackRightXPosInches = -11.875;
        private static final double kBackRightYPosInches = -8.875;

        private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
                kInvertLeftSide);
        private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches),
                kInvertRightSide);
        private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
        private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
                kInvertRightSide);

        public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants,
                FrontLeft,
                FrontRight, BackLeft, BackRight);
    }

}
