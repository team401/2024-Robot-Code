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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.localization.Camera.CameraTrustZone;
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

    // public static final Mode currentMode = Mode.REPLAY;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final class FeatureFlags {
        public static final boolean runVision = true;

        public static final boolean runIntake = true;
        public static final boolean runScoring = false;
        public static final boolean runEndgame = true;
        public static final boolean runDrive = true;

        public static final boolean enableLEDS = true;
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
        public static final int indexerSensorPort = 1;
        public static final int uptakeSensorPort = 4;
    }

    public static final class DriveConstants {
        public static final double MaxSpeedMetPerSec = 6;
        public static final double MaxAngularRateRadiansPerSec = Math.PI * 2; // 2 PI is one full
        // rotation per second
        public static final double deadbandPercent = 0.16;
        public static final double maxAccelerationMetersPerSecSquared = 7.0;

        public static final Pose2d initialPose =
                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90));

        public static final double anticipationTime = 0.01;
        public static final double minimumAnticipationVelocity = 0.0;

        public static final double pathfindTransformToleranceMeters = 0.1;
        public static final double pathfindRotationToleranceRadians = 0.1;

        public static final double alignToleranceRadians = 0.1;

        public static final double alignmentkPMax = 7.0; // 15
        public static final double alignmentkPMin = 5.0; // 8
        public static final double alignmentkI = 5.5; // 25
        public static final double alignmentkD = 0.0; // 0.4

        public static final double autoAlignmentkP = 5.0;
        public static final double autoAlignmentkI = 5.5;
        public static final double autoAlignmentkD = 0.0;

        public static final double vXkP = 5.0;
        public static final double vXkI = 2.5;
        public static final double vXkD = 0.0;

        public static final double vYkP = 5.0;
        public static final double vYkI = 2.5;
        public static final double vYkD = 0.0;
    }

    public static final class FieldConstants {
        public static final double lengthM = 16.451;
        public static final double widthM = 8.211;

        public static final double midfieldLowThresholdM = 5.87;
        public static final double midfieldHighThresholdM = 10.72;

        public static final Rotation2d ampHeading = new Rotation2d(-Math.PI / 2);

        public static final Rotation2d blueUpHeading = Rotation2d.fromRadians(0.0);
        public static final Rotation2d blueDownHeading = Rotation2d.fromRadians(Math.PI);
        public static final Rotation2d blueLeftHeading = Rotation2d.fromRadians(Math.PI / 2.0);
        public static final Rotation2d blueRightHeading = Rotation2d.fromRadians(-Math.PI / 2.0);

        public static final Rotation2d redUpHeading = Rotation2d.fromRadians(Math.PI);
        public static final Rotation2d redDownHeading = Rotation2d.fromRadians(0.0);
        public static final Rotation2d redLeftHeading = Rotation2d.fromRadians(-Math.PI / 2.0);
        public static final Rotation2d redRightHeading = Rotation2d.fromRadians(Math.PI / 2.0);

        public static final Rotation2d redSourceHeading =
                new Rotation2d(Math.PI * 4 / 3); // 60 degrees
        public static final Rotation2d blueSourceHeading =
                new Rotation2d(Math.PI * 5 / 3); // 120 degrees

        public static final Translation2d fieldToRedSpeaker =
                new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42));

        public static final Translation2d fieldToBlueSpeaker =
                new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42));

        public static final Pose2d robotAgainstBlueSpeaker =
                new Pose2d(1.39, 5.56, Rotation2d.fromDegrees(180));

        public static final Pose2d robotAgainstRedSpeaker =
                new Pose2d(15.19, 5.56, Rotation2d.fromDegrees(0));

        public static final Pose2d robotAgainstBlueSpeakerRight =
                new Pose2d(0.7, 4.38, Rotation2d.fromDegrees(120));

        public static final Pose2d robotAgainstRedSpeakerRight =
                new Pose2d(15.83, 6.73, Rotation2d.fromDegrees(-60));

        public static final Pose2d robotAgainstBlueSpeakerLeft =
                new Pose2d(0.7, 6.73, Rotation2d.fromDegrees(-120));

        public static final Pose2d robotAgainstRedSpeakerLeft =
                new Pose2d(15.83, 4.38, Rotation2d.fromDegrees(60));

        public static final Pose2d robotAgainstBluePodium =
                new Pose2d(2.57, 4.09, Rotation2d.fromDegrees(180));

        public static final Pose2d robotAgainstRedPodium =
                new Pose2d(13.93, 4.09, Rotation2d.fromDegrees(0));

        public static final Pose2d robotAgainstBlueAmpZone =
                new Pose2d(2.85, 7.68, Rotation2d.fromDegrees(-90));

        public static final Pose2d robotAgainstRedAmpZone =
                new Pose2d(13.74, 7.68, Rotation2d.fromDegrees(-90));
    }

    public static final class VisionConstants {
        public static final String tagLayoutName = "Pairs-Only";
        public static final AprilTagFieldLayout fieldLayout = initLayout(tagLayoutName);

        public static final double lowUncertaintyCutoffDistance = 6.5;

        public static final double skewCutoffDistance = 5.8;
        public static final double skewCutoffRotation = Units.degreesToRadians(50);

        public static final Matrix<N3, N1> teleopCameraUncertainty =
                VecBuilder.fill(0.35, 0.35, 3.5);

        public static final Matrix<N3, N1> lowCameraUncertainty = VecBuilder.fill(0.6, 1.0, 4);

        public static final Matrix<N3, N1> highCameraUncertainty = VecBuilder.fill(12.0, 16.0, 40);

        public static final Matrix<N3, N1> driveUncertainty = VecBuilder.fill(0.1, 0.1, 0.1);

        public static final List<CameraParams> cameras =
                List.of(
                        new CameraParams(
                                "Front-Left",
                                1280,
                                800,
                                16,
                                Rotation2d.fromDegrees(70),
                                new Transform3d(
                                        new Translation3d(0.306, 0.259, 0.211),
                                        new Rotation3d(0, -0.349, 0.785)),
                                CameraTrustZone.LEFT),
                        new CameraParams(
                                "Front-Right",
                                1280,
                                800,
                                16,
                                Rotation2d.fromDegrees(70),
                                new Transform3d(
                                        new Translation3d(0.312, -0.304, 0.217),
                                        new Rotation3d(0.0, -0.349, -0.785)),
                                CameraTrustZone.RIGHT),
                        new CameraParams(
                                "Front-Center",
                                1280,
                                800,
                                16,
                                Rotation2d.fromDegrees(70),
                                new Transform3d(
                                        new Translation3d(0.312, -0.237, 0.233),
                                        new Rotation3d(0.0, -0.349, 0.524)),
                                CameraTrustZone.MIDDLE));

        public static record CameraParams(
                String name,
                int xResolution,
                int yResolution,
                int fps,
                Rotation2d fov,
                Transform3d robotToCamera,
                CameraTrustZone zone) {}

        private static AprilTagFieldLayout initLayout(String name) {
            AprilTagFieldLayout layout;
            // AprilTagFieldLayout's constructor throws an IOException, so we must catch it
            // in order to initialize our layout as a static constant
            try {
                layout =
                        new AprilTagFieldLayout(
                                Filesystem.getDeployDirectory().getAbsolutePath()
                                        + "/taglayout/"
                                        + name
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
        public static final int leftIntakeMotorID = 9;
        public static final int rightIntakeMotorID = 10;
        public static final int indexTwoMotorID = 14;

        public static final double intakePower = 12.0;
        public static final double beltPower = 12.0;
    }

    public static final class EndgameConstants {
        public static final int leftMotorID = 18;
        public static final int rightMotorID = 19;

        public static final int smartCurrentLimit = 200;

        public static final TrapezoidProfile.Constraints climberProfileConstraints =
                new TrapezoidProfile.Constraints(0.5, 0.3); // TODO: Find safe values for this

        // Values used to clamp target position of the climber (could potentially be identical to
        // our target positions)
        public static final double climberMinPositionMeters = 0.0;
        public static final double climberMaxPositionMeters = 0.52;

        // this could be negative if we want to rely on our current limit
        public static final double climberTargetDownMeters = 0.0;
        public static final double climberTargetUpMeters = 0.52;

        // TODO: Actual mass of robot
        public static final double simRobotMass = 53.0703; // 117 pounds to kg

        // Constants for the climber PID controller
        public static final double climberkP = 7.0; // TODO: Tune climber PID controller
        public static final double climberkI = 0.0;
        public static final double climberkD = 0.0;

        // TODO: Tune feedforward values for climber
        // Feedforward value for raising elevator
        public static final double climberkFFClimber = 0.125;
        // Feedforward value for lowering elevator, assuming we are now lifting entire
        // weight of the robot
        public static final double climberkFFRobot = -0.5;

        public static final double encoderToMeters = Math.PI * Units.inchesToMeters(1.7567) / 20.0;
    }

    public static final class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        public static final Slot0Configs steerGains =
                new Slot0Configs()
                        .withKP(150)
                        .withKI(50)
                        .withKD(0.2)
                        .withKS(0.25)
                        .withKV(1.5)
                        .withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        public static final Slot0Configs driveGains =
                new Slot0Configs()
                        .withKP(0)
                        .withKI(0.02)
                        .withKD(0)
                        .withKS(0.26)
                        .withKV(0.12)
                        .withKA(0.01);

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
        private static final double kSlipCurrentA = 80;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 5.02; // 5.21 OR 5.02

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.122448979591837;
        private static final double kSteerGearRatio = 21.428571428571427;
        private static final double kWheelRadiusInches = 1.931;

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
        private static final int kBackRightDriveMotorId = 2;
        private static final int kBackRightSteerMotorId = 1;
        private static final int kBackRightEncoderId = 1;
        private static final double kBackRightEncoderOffset = 0.3486328125;

        private static final double kBackRightXPosInches = 10.375;
        private static final double kBackRightYPosInches = 10.375;

        // Front Right
        private static final int kBackLeftDriveMotorId = 4;
        private static final int kBackLeftSteerMotorId = 3;
        private static final int kBackLeftEncoderId = 2;
        private static final double kBackLeftEncoderOffset = 0.096435546875;

        private static final double kBackLeftXPosInches = 10.375;
        private static final double kBackLeftYPosInches = -10.375;

        // Back Left
        private static final int kFrontRightDriveMotorId = 8;
        private static final int kFrontRightSteerMotorId = 7;
        private static final int kFrontRightEncoderId = 4;
        private static final double kFrontRightEncoderOffset = 0.130859375;

        private static final double kFrontRightXPosInches = -10.375;
        private static final double kFrontRightYPosInches = 10.375;

        // Back Right
        private static final int kFrontLeftDriveMotorId = 6;
        private static final int kFrontLeftSteerMotorId = 5;
        private static final int kFrontLeftEncoderId = 3;
        private static final double kFrontLeftEncoderOffset = -0.372802734375;

        private static final double kFrontLeftXPosInches = -10.375;
        private static final double kFrontLeftYPosInches = -10.375;

        public static final double kModuleRadiusMeters =
                Units.inchesToMeters(Math.hypot(kFrontLeftXPosInches, kFrontLeftYPosInches));

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
        public static final double aimerkP = 17.0;
        public static final double aimerkI = 10.0; // 5.0
        public static final double aimerkD = 0.0;

        public static final double aimerkS = 0.265;
        public static final double aimerkG = 0.1;
        public static final double aimerkV = 1.51;
        public static final double aimerkA = 0.01;

        public static final double shooterkP = 0.05;
        public static final double shooterkI = 0.2;
        public static final double shooterkD = 0.0;

        public static final double shooterkS = 0.0; // TODO: Find Imperically
        public static final double shooterkV = 0.0095;
        public static final double shooterkA = 0.0;

        public static final double hoodkP = 0.05;
        public static final double hoodkI = 0.0;
        public static final double hoodkD = 0.0;

        public static final double hoodkS = 0.14; // 0.14
        public static final double hoodkG = 0.41; // 0.41
        public static final double hoodkV = 0.0;

        public static final double hoodPositionTolerance = 0.01;

        public static final double hoodEncoderToRad = 1.3 * (15.0 / 38.0) * (2.0 * Math.PI);

        public static final int aimLeftMotorId = 16;
        public static final int aimRightMotorId = 15;

        public static final int shooterLeftMotorId = 11;
        public static final int shooterRightMotorId = 12;

        public static final int kickerMotorId = 13;

        public static final int hoodId = 17;

        public static final int aimEncoderPort = 0;
        public static final double aimerEncoderOffset = 1.75 - 0.01; // 0.027

        public static final double kickerIntakeVolts = 2.0;

        public static final double aimPositionTolerance = 0.017;

        public static final double aimAcceleration = 4.5; // TODO: 15.0
        public static final double aimCruiseVelocity = 7.0; // TODO: 15.0

        public static final double shooterLowerVelocityMarginRPM = 50;
        public static final double shooterUpperVelocityMarginRPM = 150;
        public static final double aimAngleMarginRadians = Units.degreesToRadians(1);
        public static final double aimAngleVelocityMargin = 2.0; // Units.degreesToRadians(5);
        public static final double hoodAngleMarginRadians = Units.degreesToRadians(5);

        public static final double intakeAngleToleranceRadians = 0.2;
        // Math.PI / 2 - Units.degreesToRadians(40);

        public static final double shooterAmpVelocityRPM = 2000;

        public static final double hoodHomeAmps = 40.0; // TODO: Find this
        public static final double hoodHomeAngleRad = Math.PI - 0.23;

        public static final double aimMaxAngleRadians = 1.65; // Math.PI / 2
        public static final double aimMinAngleRadians = -0.03;

        public static final double maxAimIntake = 0.0;
        public static final double minAimIntake = 0.0;

        public static final double shooterOffsetAdjustment = 0.6;

        public static final double maxElevatorPosition = 0.45;
        public static final double maxAimAngleElevatorLimit = Math.PI / 2;

        public static final double hoodMaxVelocity = 0.5;
        public static final double hoodMaxAcceleration = 0.5;

        // NOTE - This should be monotonically increasing
        // Key - Distance in meters
        // Value - Aimer angle in radians
        public static HashMap<Double, Double> getAimerMap() {
            HashMap<Double, Double> map = new HashMap<Double, Double>();
            map.put(0.0, 0.8);
            map.put(1.45, 0.8);
            map.put(1.98, 0.62);
            map.put(2.41, 0.53);
            map.put(3.02, 0.45);
            map.put(3.22, 0.45);
            map.put(3.9, 0.36);
            map.put(4.55, 0.35);
            map.put(4.95, 0.32);
            map.put(5.15, 0.295);
            map.put(5.35, 0.295);
            map.put(5.5, 0.295);
            map.put(5.64, 0.29);
            // map.put(5.82, 0.275);
            map.put(6.0, 0.29);

            return map;
        }

        public static final double aimerStaticOffset = 0.01;

        // NOTE - This should be monotonically increasing
        // Key - Distance in meters
        // Value - Shooter RPM
        public static HashMap<Double, Double> getShooterMap() {
            HashMap<Double, Double> map = new HashMap<Double, Double>();
            map.put(0.0, 2700.0);
            map.put(1.45, 2700.0);
            map.put(1.98, 2700.0);
            map.put(2.41, 3000.0);
            map.put(3.02, 3300.0);
            map.put(3.22, 3300.0);
            map.put(3.9, 3300.0);
            map.put(4.55, 3500.0);
            map.put(4.95, 3500.0); // 4000.0
            map.put(5.15, 3500.0);
            map.put(5.35, 3500.0);
            map.put(5.64, 3500.0);
            map.put(5.82, 3500.0);

            return map;
        }

        // NOTE - This should be monotonically increasing
        // Key - Distance in meters
        // Value - Time in seconds
        public static HashMap<Double, Double> timeToGoalMap() {
            HashMap<Double, Double> map = new HashMap<Double, Double>();
            map.put(0.0, 0.15);
            map.put(1.3, 0.15);
            map.put(1.4, 0.16);
            // map.put(1.3, 0.15);

            return map;
        }

        // NOTE - This should be monotonically increasing
        // Key - Angle in radians
        // Value - Time in seconds
        public static HashMap<Double, Double> timeToPutAimDownMap() { // TODO: Find this
            HashMap<Double, Double> map = new HashMap<Double, Double>();
            map.put(0.0, 0.6);
            map.put(Math.PI / 6, 0.6);
            map.put(Math.PI / 4, 0.7);
            map.put(Math.PI / 3, 0.8);
            map.put(Math.PI / 2, 1.0);

            return map;
        }

        // NOTE - This should be monotonically increasing
        // Key - Elevator position in meters
        // Value - Aimer angle in radians
        public static HashMap<Double, Double> aimerAvoidElevatorTable() {
            HashMap<Double, Double> map = new HashMap<Double, Double>();
            map.put(0.0, 0.0);
            map.put(0.01, Math.PI / 8);
            map.put(0.05, Math.PI / 6);
            map.put(0.1, Math.PI / 4);
            map.put(0.2, Math.PI / 3);
            map.put(0.4, 1.37);

            return map;
        }

        // NOTE - This should be monotonically increasing
        // Key - Distance to goal in meters
        // Value - Aimer angle tolerance in radians
        public static HashMap<Double, Double> aimerToleranceTable() {
            HashMap<Double, Double> map = new HashMap<Double, Double>();
            map.put(0.0, 0.1);
            map.put(2.0, 0.1);
            map.put(10.0, 0.015);

            return map;
        }
    }

    public static final class LEDConstants {
        public static final int ledPort = 0;

        public static final int ledLength = 17;
    }

    public static final class IOConstants {
        public static final int brakeSwitchPort = 2;
        public static final int ledSwitchPort = 3;
        public static final int timeOutputPort = 4;
    }
}
