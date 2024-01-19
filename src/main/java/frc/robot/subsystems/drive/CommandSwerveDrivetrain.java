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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private double vx, vy, omega = 0.0;
    private boolean fieldCentric = true;

    private SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();
    private SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric();
    private SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private static final double kSimLoopPeriod = 0.005; // 5 ms
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
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Constants.currentMode == Constants.Mode.SIM) {
            startSimThread();
        }
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
                                speeds, true), // Consumer of ChassisSpeeds to drive the robot
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

    private void controlDrivetrain() {
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

    @Override
    public void periodic() {
        controlDrivetrain();
    }
}
