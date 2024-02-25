package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.InterpolateDouble;
import frc.robot.utils.Tunable;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Tunable, Subsystem {
    private double vx, vy, omega = 0.0;
    private boolean fieldCentric = true;

    private double alignError = 0.0;

    public enum AlignTarget {
        NONE,
        AMP,
        SPEAKER,
        SOURCE
    }

    public enum AlignState {
        MANUAL,
        ALIGNING,
    }

    public enum TuningMode {
        NONE,
        DRIVE,
        ROTATION,
    }

    private AlignTarget alignTarget = AlignTarget.NONE;
    private AlignState alignState = AlignState.MANUAL;

    private TuningMode tuningMode = TuningMode.NONE;
    private double tuningVolts = 0.0;

    private static InterpolateDouble noteTimeToGoal =
            new InterpolateDouble(ScoringConstants.timeToGoalMap(), 0.0, 2.0);

    private Supplier<Pose2d> getFieldToRobot = () -> new Pose2d();
    private Supplier<Translation2d> getFieldToSpeaker = () -> new Translation2d();

    private Supplier<Rotation2d> getFieldToAmp = () -> new Rotation2d();
    private Supplier<Rotation2d> getFieldToSource = () -> new Rotation2d();

    private Supplier<Translation2d> getRobotVelocity = () -> new Translation2d();

    private PIDController thetaController =
            new PIDController(
                    DriveConstants.alignmentkPMax,
                    DriveConstants.alignmentkI,
                    DriveConstants.alignmentkD);

    private SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();
    private SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric();
    private SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private SwerveRequest.SysIdSwerveSteerGains rotationSysId =
            new SwerveRequest.SysIdSwerveSteerGains();
    private SwerveRequest.SysIdSwerveTranslation driveSysId =
            new SwerveRequest.SysIdSwerveTranslation();

    private static final double kSimLoopPeriod = 0.02; // Original: 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants,
            double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Constants.currentMode == Constants.Mode.SIM) {
            startSimThread();
        }

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        /*
         * Since we're extending `SwerveDrivetrain`, we can't extend `SubsystemBase`, we can only
         * implement `Subsystem`. Because of this, we have to register ourself manaully.
         *
         * In short, never trust the Command Scheduler.
         */
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Constants.currentMode == Constants.Mode.SIM) {
            startSimThread();
        }

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        /*
         * Since we're extending `SwerveDrivetrain`, we can't extend `SubsystemBase`, we can only
         * implement `Subsystem`. Because of this, we have to register ourself manaully.
         *
         * In short, never trust the Command Scheduler.
         */
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void setPoseSupplier(Supplier<Pose2d> getFieldToRobot) {
        this.getFieldToRobot = getFieldToRobot;
    }

    public void setSpeakerSupplier(Supplier<Translation2d> getFieldToSpeaker) {
        this.getFieldToSpeaker = getFieldToSpeaker;
    }

    public void setAmpSupplier(Supplier<Rotation2d> getFieldToAmp) {
        this.getFieldToAmp = getFieldToAmp;
    }

    public void setSourceSupplier(Supplier<Rotation2d> getFieldToSource) {
        this.getFieldToSource = getFieldToSource;
    }

    public void setAlignTarget(AlignTarget alignTarget) {
        this.alignTarget = alignTarget;
    }

    public void setAlignState(AlignState state) {
        this.alignState = state;
    }

    public AlignState getAlignState() {
        return alignState;
    }

    public void setVelocitySupplier(Supplier<Translation2d> getRobotVelocity) {
        this.getRobotVelocity = getRobotVelocity;
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        Pathfinding.setPathfinder(new LocalADStar());

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> {
                    this.setGoalChassisSpeeds(speeds, false);
                    this.setAlignState(AlignState.ALIGNING);
                    this.setAlignTarget(AlignTarget.SPEAKER);
                }, // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(
                        new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier =
                new Notifier(
                        () -> {
                            final double currentTime = Utils.getCurrentTimeSeconds();
                            double deltaTime = currentTime - lastSimTime;
                            lastSimTime = currentTime;

                            // System.out.println(System.currentTimeMillis());

                            /* use the measured time delta, get battery voltage from WPILib */
                            updateSimState(deltaTime, RobotController.getBatteryVoltage());
                        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void setGoalChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean fieldCen) {
        vx = chassisSpeeds.vxMetersPerSecond;
        vy = chassisSpeeds.vyMetersPerSecond;
        omega = chassisSpeeds.omegaRadiansPerSecond;
        fieldCentric = fieldCen;
    }

    public void setGoalChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setGoalChassisSpeeds(chassisSpeeds, true);
    }

    private void controlDrivetrain() {
        Pose2d pose = getFieldToRobot.get();
        Rotation2d desiredHeading = pose.getRotation();
        if (alignState == AlignState.ALIGNING) {
            switch (alignTarget) {
                case AMP:
                    desiredHeading = getFieldToAmp.get();
                    break;
                case SPEAKER:
                    desiredHeading =
                            calculateDesiredHeading(
                                    pose, new Pose2d(getFieldToSpeaker.get(), new Rotation2d()));
                    break;
                case SOURCE:
                    desiredHeading = getFieldToSource.get();
                    break;
                case NONE:
                    break;
                default:
                    break;
            }

            alignError = thetaController.getPositionError();

            double t = Math.abs(alignError) / (Math.PI / 4);
            double minkP = DriveConstants.alignmentkPMin;
            double maxkP = DriveConstants.alignmentkPMax;
            double rotationkP = minkP * (1.0 - t) + t * maxkP;

            // double rotationkP1 = (maxkP-minkP)/(Math.PI/4) * (alignError) + 0.1;

            if (t > 1) { //
                rotationkP = maxkP;
            }

            Logger.recordOutput("thetaController/rotationkP", rotationkP);
            thetaController.setP(rotationkP);

            omega =
                    -thetaController.calculate(
                            pose.getRotation().getRadians(), desiredHeading.getRadians());

            Logger.recordOutput("Drive/omegaCommand", omega);
            Logger.recordOutput("Drive/desiredHeading", desiredHeading.getRadians());
            Logger.recordOutput("Drive/rotationError", thetaController.getPositionError());
        }

        Logger.recordOutput("Drive/alignState", alignState);
        Logger.recordOutput("Drive/alignTarget", alignTarget);
        Logger.recordOutput("Drive/tuningMode", tuningMode);
        Logger.recordOutput("Drive/desiredHeading", desiredHeading);
        Logger.recordOutput("Drive/fieldToSpeaker", getFieldToSpeaker.get());
        Logger.recordOutput("Drive/goalChassisSpeeds", new ChassisSpeeds(vx, vy, omega));

        switch (tuningMode) {
            case NONE:
                if (!fieldCentric) {
                    setControl(
                            driveRobotCentric
                                    .withVelocityX(vx)
                                    .withVelocityY(vy)
                                    .withRotationalRate(omega)
                                    .withDeadband(0.0)
                                    .withRotationalDeadband(0.0));
                } else {
                    setControl(
                            driveFieldCentric
                                    .withVelocityX(vx)
                                    .withVelocityY(vy)
                                    .withRotationalRate(omega)
                                    .withDeadband(0.0)
                                    .withRotationalDeadband(0.0));
                }
                break;
            case DRIVE:
                setControl(driveSysId.withVolts(Units.Volts.of(tuningVolts)));
                break;
            case ROTATION:
                setControl(rotationSysId.withVolts(Units.Volts.of(tuningVolts)));
                break;
        }
    }

    private Rotation2d calculateDesiredHeading(Pose2d current, Pose2d target) {
        Translation2d robotVelocityAdjusted =
                getRobotVelocity.get().times(DriveConstants.anticipationTime);

        if (robotVelocityAdjusted.getNorm() < DriveConstants.minimumAnticipationVelocity) {
            robotVelocityAdjusted = new Translation2d(0, 0);
        }

        double robotXAnticipated = current.getX() + robotVelocityAdjusted.getX();
        double robotYAnticipated = current.getY() + robotVelocityAdjusted.getY();

        Pose2d robotAnticipated =
                new Pose2d(robotXAnticipated, robotYAnticipated, current.getRotation());

        Pose2d robotToTargetAnticipated = GeomUtil.transformToPose(robotAnticipated.minus(target));

        double distanceToTarget =
                Math.hypot(
                        robotToTargetAnticipated.getTranslation().getX(),
                        robotToTargetAnticipated.getTranslation().getY());

        Rotation2d angle =
                Rotation2d.fromRadians(
                        Math.atan2(
                                robotToTargetAnticipated.getY(), robotToTargetAnticipated.getX()));
        angle = angle.plus(Rotation2d.fromDegrees(180));

        double timeToGoal = noteTimeToGoal.getValue(distanceToTarget);
        double noteVelocity = distanceToTarget / timeToGoal;

        // Correction angle accounting for robot velocity
        double phi = (Math.PI / 2) - Math.acos(getRobotVelocity.get().getY() / noteVelocity);

        return angle.minus(new Rotation2d(phi));
    }

    public boolean isAligned() {
        return Math.abs(alignError) < DriveConstants.alignToleranceRadians;
    }

    @Override
    public void periodic() {
        controlDrivetrain();
    }

    @Override
    public double getPosition(int slot) {
        // probe module 1's position to have a consistent output
        switch (slot) {
            case 0:
                return getModule(1).getPosition(false).distanceMeters;
            case 1:
                return getModule(1).getPosition(false).angle.getRadians();
            default:
                throw new UnsupportedOperationException("getVelocity(): drive has only two slots");
        }

    }

    @Override
    public double getVelocity(int slot) {
        double avgVel = 0.0;
        switch (slot) {
            case 0:
                for (int i = 0; i < 4; i++) {
                    avgVel += getModule(i).getCurrentState().speedMetersPerSecond;
                }
                return avgVel / 4;
            case 1:
                for (int i = 0; i < 4; i++) {
                    avgVel += getModule(i).getCurrentState().speedMetersPerSecond;
                }
                return avgVel / 4;
            default:
                throw new UnsupportedOperationException("getVelocity(): drive has only two slots");
        }
    }

    @Override
    public double getConversionFactor(int slot) {
        return 1.0;
    }

    @Override
    public void setVolts(double volts, int slot) {
        switch (slot) {
            case 0:
                tuningMode = TuningMode.DRIVE;
                tuningVolts = volts;
                break;
            case 1:
                tuningMode = TuningMode.ROTATION;
                tuningVolts = volts;
                break;
        }
    }

    @Override
    public void setPID(double p, double i, double d, int slot) {
        Slot0Configs configs;
        switch (slot) {
            case 0:
                configs =
                        new Slot0Configs()
                                .withKP(p)
                                .withKI(i)
                                .withKD(d)
                                .withKS(TunerConstants.driveGains.kS)
                                .withKV(TunerConstants.driveGains.kV)
                                .withKA(TunerConstants.driveGains.kA);
                for (int j = 0; j < 4; j++) {
                    getModule(0).getDriveMotor().getConfigurator().apply(configs);
                }
                break;
            case 1:
                configs =
                        new Slot0Configs()
                                .withKP(p)
                                .withKI(i)
                                .withKD(d)
                                .withKS(TunerConstants.steerGains.kS)
                                .withKV(TunerConstants.steerGains.kV)
                                .withKA(TunerConstants.steerGains.kA);
                for (int j = 0; j < 4; j++) {
                    getModule(0).getSteerMotor().getConfigurator().apply(configs);
                }
                break;
        }
    }

    @Override
    public void setFF(double kS, double kV, double kA, double kG, int slot) {
        Slot0Configs configs;
        switch (slot) {
            case 0:
                configs =
                        new Slot0Configs()
                                .withKP(TunerConstants.driveGains.kP)
                                .withKI(TunerConstants.driveGains.kI)
                                .withKD(TunerConstants.driveGains.kD)
                                .withKS(kS)
                                .withKV(kV)
                                .withKA(kA);
                for (int i = 0; i < 4; i++) {
                    getModule(0).getDriveMotor().getConfigurator().apply(configs);
                }
                break;
            case 1:
                configs =
                        new Slot0Configs()
                                .withKP(TunerConstants.steerGains.kP)
                                .withKI(TunerConstants.steerGains.kI)
                                .withKD(TunerConstants.steerGains.kD)
                                .withKS(kS)
                                .withKV(kV)
                                .withKA(kA);
                for (int i = 0; i < 4; i++) {
                    getModule(0).getSteerMotor().getConfigurator().apply(configs);
                }
                break;
        }
    }

    @Override
    public void setMaxProfileProperties(double maxVelocity, double maxAcceleration, int slot) {
        throw new UnsupportedOperationException("drive does not use a motion profile");
    }

    @Override
    public void runToPosition(double position, int slot) {
        throw new UnsupportedOperationException("drive does not support PID tuning");
    }
}
