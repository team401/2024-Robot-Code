package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.InterpolateDouble;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private double vx, vy, omega = 0.0;
    private boolean fieldCentric = true;

    private double alignError = 0.0;

    private Timer elapsedDriveToNote = new Timer();
    private double estimatedTimeToNote = 0;
    private boolean drivingToNote = false;
    private double allotedErrorTime = 2;

    public enum AlignTarget {
        NONE,
        NOTE,
        AMP,
        SPEAKER,
        SOURCE
    }

    public enum AlignState {
        MANUAL,
        ALIGNING,
    }

    private AlignTarget alignTarget = AlignTarget.NONE;
    private AlignState alignState = AlignState.MANUAL;

    private PhotonCamera colorCamera = new PhotonCamera("photonvision-orange");

    private static InterpolateDouble noteTimeToGoal =
            new InterpolateDouble(ScoringConstants.timeToGoalMap(), 0.0, 2.0);

    private Supplier<Pose2d> getFieldToRobot = () -> new Pose2d();
    private Supplier<Translation2d> getFieldToSpeaker = () -> new Translation2d();

    private Supplier<Rotation2d> getFieldToAmp = () -> new Rotation2d();
    private Supplier<Rotation2d> getFieldToSource = () -> new Rotation2d();

    private Supplier<Translation2d> getRobotVelocity = () -> new Translation2d();

    private Supplier<Boolean> hasNote = () -> false;

    private PIDController thetaController =
            new PIDController(
                    DriveConstants.alignmentkPMax,
                    DriveConstants.alignmentkI,
                    DriveConstants.alignmentkD);

    private SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();
    private SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric();
    // private SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

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
        elapsedDriveToNote.reset();
        elapsedDriveToNote.stop();
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

    public void setNoteAcquiredSupplier(Supplier<Boolean> hasNote) {
        this.hasNote = hasNote;
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

    private double[] getCoordinateOfTargetCorner(TargetCorner corner) {
        String stringcorner = corner.toString();
        stringcorner = stringcorner.substring(1, stringcorner.length());
        String[] stringResult = stringcorner.split(",");
        return new double[] {
            Double.parseDouble(stringResult[0]), Double.parseDouble(stringResult[1])
        };
    }

    private Pose2d getNotePoseFromTarget(PhotonTrackedTarget target) {
        double transformWidth = 1 * 1; // = distance at setpoint / width at setpoint
        // TODO: GET PROPER D0 and H0 VALUES

        List<TargetCorner> corners = target.getDetectedCorners();

        double[][] cornersCoordinates = new double[4][2];
        for (int i = 0; i < 4; i++) {
            cornersCoordinates[i] = getCoordinateOfTargetCorner(corners.get(i));
        }

        double width = cornersCoordinates[3][0] - cornersCoordinates[2][0];
        double distanceForward = transformWidth * width;
        // alternatively: instead of using magnification maybe try using just y instead?

        double distanceSide =
                (cornersCoordinates[3][0] + cornersCoordinates[2][0]) / 2 / transformWidth;

        double distance =
                Math.sqrt(distanceForward * distanceForward + distanceSide * distanceSide);
        double angle = Math.tanh(distanceSide / distanceForward);

        Rotation2d fromForward = this.getState().Pose.getRotation().plus(new Rotation2d(angle));

        return new Pose2d(
                this.getState().Pose.getX() + Math.cos(fromForward.getRadians()) * distance,
                this.getState().Pose.getY() + Math.sin(fromForward.getRadians()) * distance,
                fromForward);
    }

    private void controlDrivetrain() {
        Pose2d pose = getFieldToRobot.get();
        Rotation2d desiredHeading = pose.getRotation();
        if (alignState != AlignState.ALIGNING || alignTarget != AlignTarget.NOTE) {
            drivingToNote = false;
            elapsedDriveToNote.stop();
            elapsedDriveToNote.reset();
        }
        if (alignState == AlignState.ALIGNING) {
            switch (alignTarget) {
                case NOTE:
                    var cameraResult = colorCamera.getLatestResult();
                    if (cameraResult.hasTargets()) {
                        Pose2d notePose = getNotePoseFromTarget(cameraResult.getBestTarget());

                        if (!drivingToNote) {

                            double distance =
                                    Math.sqrt(
                                            Math.pow(notePose.getX(), 2)
                                                    + Math.pow(notePose.getY(), 2));

                            drivingToNote = true;
                            estimatedTimeToNote =
                                    distance / DriveConstants.MaxSpeedMetPerSec + allotedErrorTime;
                            elapsedDriveToNote.reset();
                            elapsedDriveToNote.start();

                            setGoalChassisSpeeds(
                                    new ChassisSpeeds(DriveConstants.MaxSpeedMetPerSec, 0, 0),
                                    false);
                            desiredHeading = notePose.getRotation();
                        } else if (elapsedDriveToNote.get() > estimatedTimeToNote
                                || hasNote.get()) {
                            setGoalChassisSpeeds(new ChassisSpeeds());
                        } else {
                            setGoalChassisSpeeds(
                                    new ChassisSpeeds(DriveConstants.MaxSpeedMetPerSec, 0, 0),
                                    false);
                            desiredHeading = notePose.getRotation();
                        }
                    }
                    break;
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
        Logger.recordOutput("Drive/desiredHeading", desiredHeading);
        Logger.recordOutput("Drive/fieldToSpeaker", getFieldToSpeaker.get());
        Logger.recordOutput("Drive/goalChassisSpeeds", new ChassisSpeeds(vx, vy, omega));

        // if (vx == 0 && vy == 0 && omega == 0) {
        //     setControl(brake);
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

        List<TargetCorner> corners =
                colorCamera.getLatestResult().getBestTarget().getDetectedCorners();
        double[][] cornersCoordinates = new double[4][2];
        for (int i = 0; i < 4; i++) {
            cornersCoordinates[i] = getCoordinateOfTargetCorner(corners.get(i));
        }
        SmartDashboard.putNumberArray("coordinates 0", cornersCoordinates[0]);
        SmartDashboard.putNumberArray("coordinates 1", cornersCoordinates[1]);
        SmartDashboard.putNumberArray("coordinates 2", cornersCoordinates[2]);
        SmartDashboard.putNumberArray("coordinates 3", cornersCoordinates[3]);
    }
}
