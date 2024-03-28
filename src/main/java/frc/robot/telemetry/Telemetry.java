package frc.robot.telemetry;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Telemetry {
    private final TelemetryIO telemetryIo;
    private final TelemetryIOInputsAutoLogged telemetryInputs = new TelemetryIOInputsAutoLogged();

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed, TelemetryIO telemetryIo) {
        this.telemetryIo = telemetryIo;
    }

    SwerveModuleState[] moduleStates =
            new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            };

    SwerveModuleState[] moduleGoalStates =
            new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            };

    double robotRotation = 0;
    double robotRotationVelocity = 0;
    double robotRotationLast = 0;

    double velocityXFiltered = 0.0;
    double velocityYFiltered = 0.0;
    LinearFilter velocityXFilter = LinearFilter.singlePoleIIR(0.1, Constants.loopTime);
    LinearFilter velocityYFilter = LinearFilter.singlePoleIIR(0.1, Constants.loopTime);
    LinearFilter accelXFilter = LinearFilter.singlePoleIIR(0.1, Constants.loopTime);
    LinearFilter accelYFilter = LinearFilter.singlePoleIIR(0.1, Constants.loopTime);

    double accelXFiltered = 0.0;
    double accelYFiltered = 0.0;
    /* Keep a reference of the last pose to calculate the speeds */
    Pose2d latestPose = new Pose2d();
    double lastTime = Utils.getCurrentTimeSeconds();

    SwerveModuleState[] latestModuleStates = new SwerveModuleState[4];

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void telemeterize(SwerveDriveState state) {
        /*
         * PSA: Do not call the Logger in this method. This method is called
         * by a separate thread, and the Logger does not support multi-threading.
         * Your robot program will crash.
         */
        /* Telemeterize the pose */
        Pose2d pose = state.Pose;

        robotRotation = pose.getRotation().getRadians();

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;

        Translation2d velocityFieldRelative =
                new Translation2d(pose.getX() - latestPose.getX(), pose.getY() - latestPose.getY())
                        .div(diffTime);

        latestPose = pose;

        double robotRotationDiff = robotRotation - robotRotationLast;
        robotRotationVelocity = robotRotationDiff / diffTime;

        robotRotationLast = robotRotation;

        velocityXFiltered = velocityXFilter.calculate(velocityFieldRelative.getX());
        velocityYFiltered = velocityYFilter.calculate(velocityFieldRelative.getY());
        accelXFiltered = accelXFilter.calculate(telemetryInputs.accelerationX);
        accelYFiltered = accelYFilter.calculate(telemetryInputs.accelerationY);

        latestModuleStates = state.ModuleStates;

        moduleGoalStates = state.ModuleTargets;
    }

    /**
     * Calls the Logger to publish telemetry data. Call this method from the same thread as
     * CommandScheduler.
     */
    public void logDataSynchronously() {
        Pose2d pose = new Pose2d(latestPose.getX(), latestPose.getY(), latestPose.getRotation());

        telemetryIo.setRobotPose(getFieldToRobot3d());
        telemetryIo.setRobotPose(pose);
        telemetryIo.setSwerveModuleStates(getModuleStates());
        telemetryIo.setSwerveModuleGoalStates(getModuleGoalStates());

        telemetryIo.setRobotRotation(robotRotation);
        telemetryIo.setRobotRotationVelocity(robotRotationVelocity);

        telemetryIo.updateInputs(telemetryInputs);
        Logger.processInputs("telemetry", telemetryInputs);
    }

    public double getRotationRadians() {
        return robotRotation;
    }

    public double getRobotRotationRadians() {
        return robotRotationVelocity;
    }

    public Pose2d getFieldToRobot() {
        return latestPose;
    }

    public Pose3d getFieldToRobot3d() {
        return new Pose3d(
                new Translation3d(latestPose.getX(), latestPose.getY(), 0),
                new Rotation3d(0, 0, getRotationRadians()));
    }

    public Translation2d getVelocity() {
        return new Translation2d(velocityXFiltered, velocityYFiltered);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            var state = latestModuleStates[i];
            states[i] =
                    new SwerveModuleState(
                            state.speedMetersPerSecond,
                            Rotation2d.fromRadians(state.angle.getRadians()));
        }
        return states;
    }

    public SwerveModuleState[] getModuleGoalStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            var state = moduleGoalStates[i];
            states[i] =
                    new SwerveModuleState(
                            state.speedMetersPerSecond,
                            Rotation2d.fromRadians(state.angle.getRadians()));
        }
        return states;
    }

    public double getVelocityX() {
        return velocityXFiltered;
    }

    public double getVelocityY() {
        return velocityYFiltered;
    }

    public double getAccelerationX() {
        return accelXFiltered;
    }

    public double getAccelerationY() {
        return accelYFiltered;
    }
}
