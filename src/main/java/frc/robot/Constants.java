package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import java.io.IOException;
import java.util.Collections;
import java.util.List;

public final class Constants {
    public static final double loopTime = 0.02;

    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : Mode.SIM;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final class CANDevices {}

    public static final class DriveConstants {
        public static final double MaxSpeedMetPerSec = 6;
        public static final double MaxAngularRateRadiansPerSec = Math.PI * 2; // 2 PI is one full
        // rotation per second
        public static final double deadbandPercent = 0.16;
    }

    public static final class FieldConstants {
        public static final double lengthM = 16.451;
        public static final double widthM = 8.211;

        public static final double midfieldLowThresholdM = 5.87;
        public static final double midfieldHighThresholdM = 10.72;
    }

    public static final class VisionConstants {
        public static final String tagLayoutName = "2024-WPI";
        public static final AprilTagFieldLayout fieldLayout = initLayout(tagLayoutName);

        public static final double singleTagAmbiguityCutoff = 0.05;

        public static final Matrix<N3, N1> lowCameraUncertainty = VecBuilder.fill(0.45, 0.45, 1);
        public static final Matrix<N3, N1> highCameraUncertainty = VecBuilder.fill(1.2, 1.2, 10);

        public static final Matrix<N3, N1> driveUncertainty = VecBuilder.fill(0.1, 0.1, 0.1);

        // TODO: set up cameras in PhotonVision
        public static final List<CameraParams> cameras =
                List.of(new CameraParams("this isn't a real camera", new Transform3d()));

        public static record CameraParams(String name, Transform3d robotToCamera) {}

        private static AprilTagFieldLayout initLayout(String name) {
            AprilTagFieldLayout layout;
            // AprilTagFieldLayout's constructor thows an IOException, so we must catch it in order
            // to initialize our layout as a static constant
            try {
                layout =
                        new AprilTagFieldLayout(
                                Filesystem.getDeployDirectory().getAbsolutePath()
                                        + "/taglayout/2024-WPI"
                                        + ".json");
            } catch (IOException ioe) {
                // TODO: print a standardized error
                layout = new AprilTagFieldLayout(Collections.emptyList(), 0.0, 0.0);
            }
            return layout;
        }
    }

    public static final class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot
        // The steer motor uses MotionMagicVoltage control
        private static final Slot0Configs steerGains =
                new Slot0Configs().withKP(7).withKI(0).withKD(0).withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses:
        // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
        // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
        private static final Slot0Configs driveGains =
                new Slot0Configs().withKP(0.7).withKI(0).withKD(3.5).withKS(0).withKV(0).withKA(0);

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 300.0;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 6.0;

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

        private static final SwerveDrivetrainConstants DrivetrainConstants =
                new SwerveDrivetrainConstants()
                        .withPigeon2Id(kPigeonId)
                        .withCANbusName(kCANbusName);

        private static final SwerveModuleConstantsFactory ConstantCreator =
                new SwerveModuleConstantsFactory()
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

        private static final SwerveModuleConstants FrontLeft =
                ConstantCreator.createModuleConstants(
                        kFrontLeftSteerMotorId,
                        kFrontLeftDriveMotorId,
                        kFrontLeftEncoderId,
                        kFrontLeftEncoderOffset,
                        Units.inchesToMeters(kFrontLeftXPosInches),
                        Units.inchesToMeters(kFrontLeftYPosInches),
                        kInvertLeftSide);
        private static final SwerveModuleConstants FrontRight =
                ConstantCreator.createModuleConstants(
                        kFrontRightSteerMotorId,
                        kFrontRightDriveMotorId,
                        kFrontRightEncoderId,
                        kFrontRightEncoderOffset,
                        Units.inchesToMeters(kFrontRightXPosInches),
                        Units.inchesToMeters(kFrontRightYPosInches),
                        kInvertRightSide);
        private static final SwerveModuleConstants BackLeft =
                ConstantCreator.createModuleConstants(
                        kBackLeftSteerMotorId,
                        kBackLeftDriveMotorId,
                        kBackLeftEncoderId,
                        kBackLeftEncoderOffset,
                        Units.inchesToMeters(kBackLeftXPosInches),
                        Units.inchesToMeters(kBackLeftYPosInches),
                        kInvertLeftSide);
        private static final SwerveModuleConstants BackRight =
                ConstantCreator.createModuleConstants(
                        kBackRightSteerMotorId,
                        kBackRightDriveMotorId,
                        kBackRightEncoderId,
                        kBackRightEncoderOffset,
                        Units.inchesToMeters(kBackRightXPosInches),
                        Units.inchesToMeters(kBackRightYPosInches),
                        kInvertRightSide);

        public static final CommandSwerveDrivetrain DriveTrain =
                new CommandSwerveDrivetrain(
                        DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }

    public static final class Scoring {
        public static final double aimerkP = 1.0;
        public static final double aimerkI = 0.0;
        public static final double aimerkD = 0.0;

        public static final double aimerkS = 0.0;
        public static final double aimerkG = 0.0;
        public static final double aimerkV = 0.0;
        public static final double aimerkA = 0.0;

        public static final double shooterkP = 1.0;
        public static final double shooterkI = 0.0;
        public static final double shooterkD = 0.0;

        public static final double shooterkS = 0.0;
        public static final double shooterkV = 0.0;
        public static final double shooterkA = 0.0;
    }
}
