package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.TunerConstants;
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
    private boolean aligning = false;

    private static InterpolateDouble noteTimeToGoal =
            new InterpolateDouble(ScoringConstants.timeToGoalMap(), 0.0, 60.0);

    private Supplier<Pose2d> getFieldToRobot = () -> new Pose2d();
    private Supplier<Translation2d> getFieldToSpeaker = () -> new Translation2d();
    private Supplier<Translation2d> getRobotVelocity = () -> new Translation2d();

    private PIDController thetaController = new PIDController(.5, 0, 0);

    private SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();
    private SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric();
    private SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

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

        thetaController.enableContinuousInput(-180, 180);
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Constants.currentMode == Constants.Mode.SIM) {
            startSimThread();
        }
        thetaController.enableContinuousInput(-180, 180);
    }

    public void setPoseSupplier(Supplier<Pose2d> getFieldToRobot) {
        this.getFieldToRobot = getFieldToRobot;
    }

    public void setSpeakerSupplier(Supplier<Translation2d> getFieldToSpeaker) {
        this.getFieldToSpeaker = getFieldToSpeaker;
    }

    public void setVelocitySupplier(Supplier<Translation2d> getRobotVelocity) {
        this.getRobotVelocity = getRobotVelocity;
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) ->
                        this.setGoalChassisSpeeds(
                                speeds), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(
                        new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> false, // Change this if the path needs to be flipped on red vs blue
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

    public void setGoalChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean fieldCen, boolean align) {
        vx = chassisSpeeds.vxMetersPerSecond;
        vy = chassisSpeeds.vyMetersPerSecond;
        omega = chassisSpeeds.omegaRadiansPerSecond;
        fieldCentric = fieldCen;
        this.aligning = align;
    }

    public void setGoalChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setGoalChassisSpeeds(chassisSpeeds, true, false);
    }

    private void controlDrivetrain() {
        if (aligning) {
            Pose2d pose = getFieldToRobot.get();
            Rotation2d desiredHeading =
                    calculateDesiredHeading(
                            pose, new Pose2d(getFieldToSpeaker.get(), new Rotation2d()));

            Logger.recordOutput("Drive/desiredHeading", desiredHeading);
            Logger.recordOutput("Drive/fieldToSpeaker", getFieldToSpeaker.get());

            omega =
                    thetaController.calculate(
                            pose.getRotation().getDegrees(), desiredHeading.getDegrees());
            Logger.recordOutput("Drive/rotationError", thetaController.getPositionError());
        }

        if (vx == 0 && vy == 0 && omega == 0) {
            setControl(brake);
        } else if (!fieldCentric) {
            setControl(
                    driveRobotCentric
                            .withVelocityX(vx)
                            .withVelocityY(vy)
                            .withRotationalRate(omega));
        } else {
            setControl(
                    driveFieldCentric
                            .withVelocityX(vx)
                            .withVelocityY(vy)
                            .withRotationalRate(omega));
        }
    }

    private Rotation2d calculateDesiredHeading(Pose2d current, Pose2d target) {
        // I hope this is all inlined
        Pose2d robotToTarget = GeomUtil.transformToPose(current.minus(target));

        double distanceToTarget =
                Math.sqrt(
                        Math.pow(robotToTarget.getTranslation().getX(), 2)
                                + Math.pow(robotToTarget.getTranslation().getY(), 2));
        Translation2d robotToTargetAnticipated =
                new Translation2d(
                        robotToTarget.getX()
                                + (getRobotVelocity.get().getX()
                                        * noteTimeToGoal.getValue(distanceToTarget)
                                        * DriveConstants.velocityScalar),
                        robotToTarget.getY()
                                + (getRobotVelocity.get().getY()
                                        * noteTimeToGoal.getValue(distanceToTarget)
                                        * DriveConstants.velocityScalar));

        Rotation2d angle =
                Rotation2d.fromRadians(
                        Math.atan2(
                                robotToTargetAnticipated.getY(), robotToTargetAnticipated.getX()));

        angle = angle.plus(Rotation2d.fromDegrees(180));

        return angle;
    }

    @Override
    public void periodic() {
        controlDrivetrain();
    }
}
