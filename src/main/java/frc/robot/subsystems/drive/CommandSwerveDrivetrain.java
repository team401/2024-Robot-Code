package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.InterpolateDouble;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private double vx, vy, omega = 0.0;
    private boolean fieldCentric = true;

    private double alignError = 0.0;

    public enum AlignTarget {
        NONE,
        AMP,
        SPEAKER,
        SOURCE,
        ENDGAME,
        UP,
        DOWN,
        LEFT,
        RIGHT
    }

    public enum AlignState {
        MANUAL,
        ALIGNING,
        POSE_TARGET
    }

    private AlignTarget alignTarget = AlignTarget.NONE;
    private AlignState alignState = AlignState.MANUAL;

    private double alignDirection = 0.0;

    private Pose2d targetTightPose;

    private static InterpolateDouble noteTimeToGoal =
            new InterpolateDouble(ScoringConstants.timeToGoalMap(), 0.0, 2.0);

    private Supplier<Pose2d> getFieldToRobot = () -> new Pose2d();

    private Supplier<Translation2d> getRobotVelocity = () -> new Translation2d();

    private SendableChooser<String> autoChooser = new SendableChooser<String>();

    private PIDController vXController =
            new PIDController(DriveConstants.vXkP, DriveConstants.vXkI, DriveConstants.vXkD);

    private PIDController vYController =
            new PIDController(DriveConstants.vYkP, DriveConstants.vYkI, DriveConstants.vYkD);

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

    private Command pathfindCommand = null;

    private Pose2d pathfindPose = new Pose2d();

    private ChassisSpeeds stopSpeeds = new ChassisSpeeds(0, 0, 0);

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

    public void setAlignTarget(AlignTarget alignTarget) {
        this.alignTarget = alignTarget;
    }

    public void setAlignState(AlignState state) {
        this.alignState = state;
    }

    public AlignState getAlignState() {
        return alignState;
    }

    public AlignTarget getAlignTarget() {
        return alignTarget;
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
                }, // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(
                        new PIDConstants(1, 0, 0),
                        new PIDConstants(1, 0, 0),
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

        // PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

        // autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("Default", "None"); // S1-W1-W2-W3
        autoChooser.addOption("Amp Side - 4 note (2 from center)", "S1-W1-C1-C2");
        autoChooser.addOption("Amp Side - 5 note (3 from center)", "S1-W1-C1-C2-C3");
        autoChooser.addOption("Amp Side - 3 note", "S1-W1-W2");
        autoChooser.addOption("Amp Side - 4 note (wing)", "S1-W1-W2-W3");
        autoChooser.addOption("Amp Side - 5 note", "S1-W1-W2-W3-C5");
        autoChooser.addOption("Center - 3 note", "S2-W2-W3");
        autoChooser.addOption("Center - 4 note (source side to center)", "S2-W2-W3-C5");
        autoChooser.addOption("Source Side - 2 note", "S3-W3");
        autoChooser.addOption("Source Side - 3 note", "S3-W3-C5");
        autoChooser.addOption("Source Side - 5 note (across)", "S3-W3-W2-W1-C1");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutoCommand() {
        return new PathPlannerAuto(autoChooser.getSelected());
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

    public void setAlignDirection(double direction, boolean allianceSpecific) {
        if (allianceSpecific) {
            if (!DriverStation.getAlliance().isPresent()) {
                alignDirection = direction;
            } else {
                switch (DriverStation.getAlliance().get()) {
                    case Blue:
                        alignDirection = direction;
                        break;
                    case Red:
                        alignDirection = direction + 3.14;
                        break;
                }
            }
        } else {
            alignDirection = direction;
        }
    }

    private void controlDrivetrain() {
        Pose2d pose = getFieldToRobot.get();
        Rotation2d desiredHeading = pose.getRotation();
        if (alignState == AlignState.ALIGNING) {
            switch (alignTarget) {
                case AMP:
                    desiredHeading = AllianceUtil.getAmpHeading();
                    break;
                case SPEAKER:
                    desiredHeading =
                            calculateDesiredHeading(
                                    pose,
                                    new Pose2d(AllianceUtil.getFieldToSpeaker(), new Rotation2d()));
                    break;
                case SOURCE:
                    desiredHeading = AllianceUtil.getSourceHeading();
                    break;
                    // case SPECIFIC_DIRECTION:
                    //     desiredHeading = Rotation2d.fromRadians(alignDirection);
                case UP:
                    if (!DriverStation.getAlliance().isPresent()) {
                        desiredHeading = FieldConstants.blueUpHeading;
                    } else {
                        switch (DriverStation.getAlliance().get()) {
                            case Blue:
                                desiredHeading = FieldConstants.blueUpHeading;
                                break;
                            case Red:
                                desiredHeading = FieldConstants.redUpHeading;
                                break;
                        }
                    }
                    break;
                case DOWN:
                    if (!DriverStation.getAlliance().isPresent()) {
                        desiredHeading = FieldConstants.blueDownHeading;
                    } else {
                        switch (DriverStation.getAlliance().get()) {
                            case Blue:
                                desiredHeading = FieldConstants.blueDownHeading;
                                break;
                            case Red:
                                desiredHeading = FieldConstants.redDownHeading;
                                break;
                        }
                    }
                    break;
                case LEFT:
                    if (!DriverStation.getAlliance().isPresent()) {
                        desiredHeading = FieldConstants.blueLeftHeading;
                    } else {
                        switch (DriverStation.getAlliance().get()) {
                            case Blue:
                                desiredHeading = FieldConstants.blueLeftHeading;
                                break;
                            case Red:
                                desiredHeading = FieldConstants.redLeftHeading;
                                break;
                        }
                    }
                    break;
                case RIGHT:
                    if (!DriverStation.getAlliance().isPresent()) {
                        desiredHeading = FieldConstants.blueRightHeading;
                    } else {
                        switch (DriverStation.getAlliance().get()) {
                            case Blue:
                                desiredHeading = FieldConstants.blueRightHeading;
                                break;
                            case Red:
                                desiredHeading = FieldConstants.redRightHeading;
                                break;
                        }
                    }
                    break;
                case NONE:
                    break;
                case ENDGAME:
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
        } else if (alignState == AlignState.POSE_TARGET) {
            vx = vXController.calculate(pose.getX(), targetTightPose.getX());
            vy = vYController.calculate(pose.getY(), targetTightPose.getY());
            omega =
                    -thetaController.calculate(
                            pose.getRotation().getRadians(),
                            targetTightPose.getRotation().getRadians());
        }

        Logger.recordOutput("Drive/alignState", alignState);
        Logger.recordOutput("Drive/alignTarget", alignTarget);
        Logger.recordOutput("Drive/desiredHeading", desiredHeading);
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

    // private Optional<Rotation2d> getRotationTargetOverride() {
    //     return Optional.of(
    //             calculateDesiredHeading(
    //                     getFieldToRobot.get(),
    //                     new Pose2d(getFieldToSpeaker.get(), new Rotation2d())));
    // }

    private Command getPathfindCommand(Pose2d targetPose) {
        pathfindPose = targetPose;

        PathConstraints constraints =
                new PathConstraints(
                        3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

        return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0, 0.0);
    }

    public boolean atPathfindPose() {
        Transform2d translationError = getFieldToRobot.get().minus(pathfindPose);
        double rotationError =
                Math.abs(
                        getFieldToRobot.get().getRotation().getRadians()
                                - pathfindPose.getRotation().getRadians());

        return translationError.getTranslation().getNorm()
                        < DriveConstants.pathfindTransformToleranceMeters
                && rotationError < DriveConstants.pathfindRotationToleranceRadians;
    }

    public void driveToPose(Pose2d targetPose) {
        this.setAlignState(AlignState.MANUAL);

        pathfindCommand = getPathfindCommand(targetPose);
        pathfindCommand.schedule();
    }

    public void stopDriveToPose() {
        if (pathfindCommand != null) {
            pathfindCommand.cancel();
        }

        setGoalChassisSpeeds(stopSpeeds, true);
    }

    public void setPoseTarget(Pose2d pose) {
        targetTightPose = pose;
    }

    public Pose2d getEndgamePose() {
        // Blue Alliance Poses
        Pose2d leftClimbPose2d = new Pose2d(4.64, 4.46, Rotation2d.fromDegrees(-60));
        Pose2d rightClimbPose2d = new Pose2d(4.67, 3.72, Rotation2d.fromDegrees(60));
        Pose2d farClimbPose2d = new Pose2d(5.35, 4.11, Rotation2d.fromDegrees(180));

        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            // Red Alliance Poses
            leftClimbPose2d = new Pose2d(11.93, 3.72, Rotation2d.fromDegrees(120));
            rightClimbPose2d = new Pose2d(11.9, 4.49, Rotation2d.fromDegrees(-120));
            farClimbPose2d = new Pose2d(11.22, 4.08, Rotation2d.fromDegrees(0));
        }

        double distanceToTargetLeft =
                Math.hypot(
                        getFieldToRobot.get().getX() - leftClimbPose2d.getX(),
                        getFieldToRobot.get().getY() - leftClimbPose2d.getY());
        double distanceToTargetRight =
                Math.hypot(
                        getFieldToRobot.get().getX() - rightClimbPose2d.getX(),
                        getFieldToRobot.get().getY() - rightClimbPose2d.getY());
        double distanceToTargetFar =
                Math.hypot(
                        getFieldToRobot.get().getX() - farClimbPose2d.getX(),
                        getFieldToRobot.get().getY() - farClimbPose2d.getY());

        Pose2d targetPose = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180));

        if (distanceToTargetLeft < distanceToTargetRight
                && distanceToTargetLeft < distanceToTargetFar) {
            targetPose = leftClimbPose2d;
        } else if (distanceToTargetRight < distanceToTargetLeft
                && distanceToTargetRight < distanceToTargetFar) {
            targetPose = rightClimbPose2d;
        } else {
            targetPose = farClimbPose2d;
        }

        return targetPose;
    }

    public void driveToSpeaker() {
        driveToPose(new Pose2d(getFieldToSpeaker.get(), new Rotation2d()));
    }

    public void driveToEndgame() {
        // Blue Alliance Poses
        Pose2d leftClimbPose2d = new Pose2d(4.64, 4.46, Rotation2d.fromDegrees(-60));
        Pose2d rightClimbPose2d = new Pose2d(4.67, 3.72, Rotation2d.fromDegrees(60));
        Pose2d farClimbPose2d = new Pose2d(5.35, 4.11, Rotation2d.fromDegrees(180));

        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            // Red Alliance Poses
            leftClimbPose2d = new Pose2d(11.9, 4.49, Rotation2d.fromDegrees(-120));
            rightClimbPose2d = new Pose2d(11.93, 3.72, Rotation2d.fromDegrees(120));
            farClimbPose2d = new Pose2d(11.22, 4.08, Rotation2d.fromDegrees(0));
        }

        double distanceToTargetLeft =
                Math.hypot(
                        getFieldToRobot.get().getX() - leftClimbPose2d.getX(),
                        getFieldToRobot.get().getY() - leftClimbPose2d.getY());
        double distanceToTargetRight =
                Math.hypot(
                        getFieldToRobot.get().getX() - rightClimbPose2d.getX(),
                        getFieldToRobot.get().getY() - rightClimbPose2d.getY());
        double distanceToTargetFar =
                Math.hypot(
                        getFieldToRobot.get().getX() - farClimbPose2d.getX(),
                        getFieldToRobot.get().getY() - farClimbPose2d.getY());

        PathPlannerPath path = null;

        if (distanceToTargetLeft < distanceToTargetRight
                && distanceToTargetLeft < distanceToTargetFar) {
            path = PathPlannerPath.fromPathFile("LeftEndgame");
        } else if (distanceToTargetRight < distanceToTargetLeft
                && distanceToTargetRight < distanceToTargetFar) {
            path = PathPlannerPath.fromPathFile("RightEndgame");
        } else {
            path = PathPlannerPath.fromPathFile("FarEndgame");
        }

        this.setAlignState(AlignState.MANUAL);

        PathConstraints constraints =
                new PathConstraints(
                        3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

        pathfindCommand = AutoBuilder.pathfindThenFollowPath(path, constraints, 0.0);
        pathfindCommand.schedule();
    }

    public boolean isAligned() {
        return Math.abs(alignError) < DriveConstants.alignToleranceRadians;
    }

    public void runWheelRadiusCharacterization(double speed) {
        setGoalChassisSpeeds(new ChassisSpeeds(0, 0, speed));
    }

    public double[] getWheelRadiusCharacterizationPosition() {
        return new double[] {
            this.getModule(0).getDriveMotor().getPosition().getValueAsDouble(),
            this.getModule(1).getDriveMotor().getPosition().getValueAsDouble(),
            this.getModule(2).getDriveMotor().getPosition().getValueAsDouble(),
            this.getModule(3).getDriveMotor().getPosition().getValueAsDouble()
        };
    }

    @Override
    public void periodic() {
        controlDrivetrain();
    }
}
