package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import java.io.IOException;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

// Time is in seconds
// Distance is in meters
// Angle is in radians
// Speed is in meters per second

public final class Constants {
    public static final double loopTime = 0.02;

    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : Mode.SIM;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final class FeatureFlags {
        public static final boolean simulateVision = false;
        public static final boolean runVision = false;
        public static final boolean runLocalizer = true;

        public static final boolean runIntake = true;
        public static final boolean runScoring = true;
        public static final boolean runEndgame = true;
        public static final boolean runDrive = true;
    }

    public static final class ConversionConstants {
        public static final double kRadiansPerSecondToRPM = 60.0 / (2.0 * Math.PI);
        public static final double kRPMToRadiansPerSecond = 1.0 / kRadiansPerSecondToRPM;

        public static final double kSecondsToMinutes = 1.0 / 60.0;
        public static final double kMinutesToSeconds = 60.0;

        public static final double kDegreesToRadians = Math.PI / 180.0;
        public static final double kRadiansToDegrees = 180.0 / Math.PI;
    }

    public static final class CANDevices {}

    public static final class SensorConstants {
        public static final int bannerPort = 1; // TODO: Change this
    }

    public static final class DriveConstants {
        public static final double MaxSpeedMetPerSec = 6;
        public static final double MaxAngularRateRadiansPerSec = Math.PI * 2; // 2 PI is one full
        // rotation per second
        public static final double deadbandPercent = 0.16;

        public static final Pose2d initialPose =
                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90));

        public static final double anticipationTime = 0.1;
    }

    public static final class FieldConstants {
        public static final double lengthM = 16.451;
        public static final double widthM = 8.211;

        public static final double midfieldLowThresholdM = 5.87;
        public static final double midfieldHighThresholdM = 10.72;

        public static final Rotation2d fieldToAmpHeading = new Rotation2d(-Math.PI / 2);

        public static final Rotation2d fieldToRedSourceHeading =
                new Rotation2d(Math.PI / 3); // 60 degrees
        public static final Rotation2d fieldToBlueSourceHeading =
                new Rotation2d(Math.PI * 2 / 3); // 120 degrees

        public static final Translation2d fieldToRedSpeaker =
                new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42));

        public static final Translation2d fieldToBlueSpeaker =
                new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42));

        /*public static Translation2d speakerPose =
        DriverStation.getAlliance().isEmpty()
                        || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? new Translation2d(
                        Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42))
                : new Translation2d(
                        Units.inchesToMeters(652.73),
                        Units.inchesToMeters(218.42)); // TODO: Might have to change these*/
    }

    public static final class VisionConstants {
        public static final String tagLayoutName = "2024-WPI";
        public static final AprilTagFieldLayout fieldLayout = initLayout(tagLayoutName);

        public static final double singleTagAmbiguityCutoff = 0.05;

        // 0.45 from 2023
        public static final Matrix<N3, N1> lowCameraUncertainty = VecBuilder.fill(1.2, 1.2, 2);
        // 1.2 from 2023
        public static final Matrix<N3, N1> highCameraUncertainty = VecBuilder.fill(3.5, 3.5, 10);
        public static final Matrix<N3, N1> singleTagUncertainty = VecBuilder.fill(20.0, 20.0, 10);

        public static final Matrix<N3, N1> driveUncertainty = VecBuilder.fill(0.1, 0.1, 0.1);

        // TODO: set up cameras in PhotonVision
        public static final List<CameraParams> cameras =
                List.of(
                        new CameraParams(
                                "FrontLeft",
                                640,
                                480,
                                20,
                                Rotation2d.fromDegrees(70),
                                new Transform3d(
                                        new Translation3d(0.33, 0.33, 0.127), new Rotation3d())),
                        new CameraParams(
                                "FrontRight",
                                640,
                                480,
                                20,
                                Rotation2d.fromDegrees(70),
                                new Transform3d(
                                        new Translation3d(0.33, -0.33, 0.127), new Rotation3d())),
                        new CameraParams(
                                "BackLeft",
                                640,
                                480,
                                20,
                                Rotation2d.fromDegrees(70),
                                new Transform3d(
                                        new Translation3d(-0.33, -0.33, 0.127),
                                        new Rotation3d(0.0, 0.0, 3.14))),
                        new CameraParams(
                                "BackRight",
                                640,
                                480,
                                20,
                                Rotation2d.fromDegrees(70),
                                new Transform3d(
                                        new Translation3d(0.33, -0.33, 0.127),
                                        new Rotation3d(0.0, 0.0, 3.14))));

        public static record CameraParams(
                String name,
                int xResolution,
                int yResolution,
                int fps,
                Rotation2d fov,
                Transform3d robotToCamera) {}

        private static AprilTagFieldLayout initLayout(String name) {
            AprilTagFieldLayout layout;
            // AprilTagFieldLayout's constructor throws an IOException, so we must catch it
            // in order to initialize our layout as a static constant
            try {
                layout =
                        new AprilTagFieldLayout(
                                Filesystem.getDeployDirectory().getAbsolutePath()
                                        + "/taglayout/2024-WPI"
                                        + ".json");
            } catch (IOException ioe) {
                DriverStation.reportWarning(
                        "Failed to load AprilTag Layout: " + ioe.getLocalizedMessage(), false);
                layout = new AprilTagFieldLayout(Collections.emptyList(), 0.0, 0.0);
            }
            return layout;
        }
    }

    public static final class IntakeConstants {
        public static final int leftIntakeMotorID = 0;
        public static final int rightIntakeMotorID = 0;
        public static final int frontBeltMotorID = 0;
        public static final int backBeltMotorID = 0;

        public static final int bannerSensorID = 0;

        public static final double intakePower = 5.0;
        public static final double beltPower = 5.0;
    }

    public static final class EndgameConstants {
        public static final int leftMotorID = 1;
        public static final int rightMotorID = 2;
        public static final int endgameUp = 3;
        public static final int endgameDown = 0;
        public static final int ticksPerFoot = 10;
    }

    public static final class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains =
                new Slot0Configs()
                        .withKP(100)
                        .withKI(0)
                        .withKD(0.2)
                        .withKS(0)
                        .withKV(1.5)
                        .withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains =
                new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput =
                ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput =
                ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 300.0;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 5.21;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.122448979591837;
        private static final double kSteerGearRatio = 21.428571428571427;
        private static final double kWheelRadiusInches = 2;

        private static final boolean kSteerMotorReversed = true;
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final String kCANbusName = "Canivore";
        private static final int kPigeonId = 1;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;

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
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                        .withSteerInertia(kSteerInertia)
                        .withDriveInertia(kDriveInertia)
                        .withSteerFrictionVoltage(kSteerFrictionVoltage)
                        .withDriveFrictionVoltage(kDriveFrictionVoltage)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withCouplingGearRatio(kCoupleRatio)
                        .withSteerMotorInverted(kSteerMotorReversed);

        // Front Left
        private static final int kFrontLeftDriveMotorId = 2;
        private static final int kFrontLeftSteerMotorId = 1;
        private static final int kFrontLeftEncoderId = 1;
        private static final double kFrontLeftEncoderOffset = 0.3486328125;

        private static final double kFrontLeftXPosInches = 10.375;
        private static final double kFrontLeftYPosInches = 10.375;

        // Front Right
        private static final int kFrontRightDriveMotorId = 4;
        private static final int kFrontRightSteerMotorId = 3;
        private static final int kFrontRightEncoderId = 2;
        private static final double kFrontRightEncoderOffset = 0.096435546875;

        private static final double kFrontRightXPosInches = 10.375;
        private static final double kFrontRightYPosInches = -10.375;

        // Back Left
        private static final int kBackLeftDriveMotorId = 8;
        private static final int kBackLeftSteerMotorId = 7;
        private static final int kBackLeftEncoderId = 4;
        private static final double kBackLeftEncoderOffset = 0.130859375;

        private static final double kBackLeftXPosInches = -10.375;
        private static final double kBackLeftYPosInches = 10.375;

        // Back Right
        private static final int kBackRightDriveMotorId = 6;
        private static final int kBackRightSteerMotorId = 5;
        private static final int kBackRightEncoderId = 3;
        private static final double kBackRightEncoderOffset = -0.372802734375;

        private static final double kBackRightXPosInches = -10.375;
        private static final double kBackRightYPosInches = -10.375;

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

        public static final SwerveDriveKinematics kinematics =
                new SwerveDriveKinematics(
                        new Translation2d(FrontLeft.LocationX, FrontLeft.LocationY),
                        new Translation2d(FrontLeft.LocationX, FrontRight.LocationY),
                        new Translation2d(BackLeft.LocationX, BackLeft.LocationY),
                        new Translation2d(BackRight.LocationX, BackRight.LocationY));

        public static final CommandSwerveDrivetrain DriveTrain =
                new CommandSwerveDrivetrain(
                        DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }

    public static final class ScoringConstants {
        public static final double aimerkP = 6.0;
        public static final double aimerkI = 0.4;
        public static final double aimerkD = 2.5;

        public static final double aimerkS = 0.0;
        public static final double aimerkG = 0.0;
        public static final double aimerkV = 0.0;
        public static final double aimerkA = 0.0;

        public static final double shooterkP = 10.0;
        public static final double shooterkI = 0.0;
        public static final double shooterkD = 0.0;

        public static final double shooterkS = 0.0;
        public static final double shooterkV = 0.0;
        public static final double shooterkA = 0.0;

        public static final double hoodkP = 0.02;
        public static final double hoodkI = 0.0;
        public static final double hoodkD = 0.6;

        public static final int aimLeftMotorId = 9;
        public static final int aimRightMotorId = 10;

        public static final int shooterLeftMotorId = 11;
        public static final int shooterRightMotorId = 12;

        public static final int kickerMotorId = 13;

        public static final int hoodId = 14;

        public static final int aimEncoderPort = 0; // TODO: Change

        public static final double aimAcceleration = 1;
        public static final double aimCruiseVelocity = 1;

        public static final double shooterAcceleration = 1;
        public static final double shooterJerk = 1;

        public static final double shooterVelocityMarginRPM = 10;
        public static final double aimAngleMarginRadians = Units.degreesToRadians(5);
        public static final double hoodAngleMarginRadians = Units.degreesToRadians(5);

        public static final double intakeAngleToleranceRadians =
                Math.PI / 2 - Units.degreesToRadians(40);

        public static final double shooterAmpVelocityRPM = 10;

        public static final double aimMaxAngleRadians = Math.PI / 2;

        public static final double maxAimIntake = 0.0;
        public static final double minAimIntake = 0.0;

        public static final double shooterOffsetAdjustment = 0.6;

        // NOTE - This should be monotonically increasing
        // Key - Distance in meters
        // Value - Aimer angle in radians
        public static HashMap<Double, Double> getAimerMap() { // TODO: Find this
            HashMap<Double, Double> map = new HashMap<Double, Double>();
            map.put(0.0, 1.0);
            map.put(1.0, 0.9);
            map.put(2.0, 0.85);
            map.put(3.0, 0.83);
            map.put(4.0, 0.64);
            map.put(5.0, 0.59);
            map.put(6.0, 0.48);
            map.put(7.0, 0.34);
            map.put(8.0, 0.27);
            map.put(9.0, 0.15);
            map.put(10.0, 0.1);

            return map;
        }

        // NOTE - This should be monotonically increasing
        // Key - Distance in meters
        // Value - Shooter RPM
        public static HashMap<Double, Double> getShooterMap() { // TODO: Find this
            HashMap<Double, Double> map = new HashMap<Double, Double>();
            map.put(0.0, 20.0);
            map.put(1.0, 40.0);
            map.put(2.0, 60.0);
            map.put(3.0, 80.0);
            map.put(4.0, 100.0);
            map.put(5.0, 120.0);
            map.put(6.0, 140.0);
            map.put(7.0, 160.0);
            map.put(8.0, 180.0);
            map.put(9.0, 190.0);
            map.put(10.0, 200.0);

            return map;
        }

        // NOTE - This should be monotonically increasing
        // Key - Distance in meters
        // Value - Time in seconds
        public static HashMap<Double, Double> timeToGoalMap() { // TODO: Find this
            HashMap<Double, Double> map = new HashMap<Double, Double>();
            map.put(0.0, 0.01);
            map.put(1.0, 0.02);
            map.put(2.0, 0.03);
            map.put(3.0, 0.04);
            map.put(4.0, 0.05);
            map.put(5.0, 0.06);
            map.put(6.0, 0.07);
            map.put(7.0, 0.1);
            map.put(8.0, 0.15);
            map.put(9.0, 0.2);
            map.put(10.0, 0.3);

            return map;
        }

        // NOTE - This should be monotonically increasing
        // Key - Angle in radians
        // Value - Time in seconds
        public static HashMap<Double, Double> timeToPutAimDownMap() { // TODO: Find this
            HashMap<Double, Double> map = new HashMap<Double, Double>();
            map.put(0.0, 0.0);
            map.put(Math.PI / 6, 0.2);
            map.put(Math.PI / 4, 0.5);
            map.put(Math.PI / 3, 0.7);
            map.put(Math.PI / 2, 1.0);

            return map;
        }
    }
}
