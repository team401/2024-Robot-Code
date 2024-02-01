package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class Telemetry {
    private final double maxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    /* What to publish over networktables for telemetry */
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot pose for field positioning */
    NetworkTable table = inst.getTable("Pose");
    DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    SwerveModuleState[] moduleStates =
            new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            };

    double robotRotation = 0;

    /* Robot speeds for general checking */
    NetworkTable driveStats = inst.getTable("Drive");
    DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
    DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
    double velocityXFieldRelative = 0.0;
    double velocityYFieldRelative = 0.0;
    DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
    DoublePublisher odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish();

    /* Keep a reference of the last pose to calculate the speeds */
    Pose2d latestPose = new Pose2d();
    double lastTime = Utils.getCurrentTimeSeconds();

    SwerveModuleState[] latestModuleStates = new SwerveModuleState[4];

    /* Mechanisms to represent the swerve module states */
    Mechanism2d[] m_moduleMechanisms =
            new Mechanism2d[] {
                new Mechanism2d(1, 1),
                new Mechanism2d(1, 1),
                new Mechanism2d(1, 1),
                new Mechanism2d(1, 1),
            };
    /* A direction and length changing ligament for speed representation */
    MechanismLigament2d[] m_moduleSpeeds =
            new MechanismLigament2d[] {
                m_moduleMechanisms[0]
                        .getRoot("RootSpeed", 0.5, 0.5)
                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
                m_moduleMechanisms[1]
                        .getRoot("RootSpeed", 0.5, 0.5)
                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
                m_moduleMechanisms[2]
                        .getRoot("RootSpeed", 0.5, 0.5)
                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
                m_moduleMechanisms[3]
                        .getRoot("RootSpeed", 0.5, 0.5)
                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
            };
    /* A direction changing and length constant ligament for module direction */
    MechanismLigament2d[] m_moduleDirections =
            new MechanismLigament2d[] {
                m_moduleMechanisms[0]
                        .getRoot("RootDirection", 0.5, 0.5)
                        .append(
                                new MechanismLigament2d(
                                        "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                m_moduleMechanisms[1]
                        .getRoot("RootDirection", 0.5, 0.5)
                        .append(
                                new MechanismLigament2d(
                                        "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                m_moduleMechanisms[2]
                        .getRoot("RootDirection", 0.5, 0.5)
                        .append(
                                new MechanismLigament2d(
                                        "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                m_moduleMechanisms[3]
                        .getRoot("RootDirection", 0.5, 0.5)
                        .append(
                                new MechanismLigament2d(
                                        "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            };

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void telemeterize(SwerveDriveState state) {
        /*
         * PSA: Do not call the Logger in this method. This method is called
         * by a separate thread, and the Logger does not support multi-threading.
         * Your robot program will crash.
         */
        /* Telemeterize the pose */
        Pose2d pose = state.Pose;
        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()});

        robotRotation = pose.getRotation().getRadians();

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(latestPose).getTranslation();

        latestPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        speed.set(velocities.getNorm());
        velocityX.set(velocities.getX());
        velocityY.set(velocities.getY());
        velocityXFieldRelative = MathUtil.clamp(velocities.getX() * 0.2, -1.5, 1.5);
        velocityYFieldRelative = MathUtil.clamp(velocities.getY() * 0.2, -1.5, 1.5);
        odomPeriod.set(state.OdometryPeriod);

        latestModuleStates = state.ModuleStates;

        /* Telemeterize the module's states */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(
                    state.ModuleStates[i].speedMetersPerSecond / (2 * maxSpeed));

            moduleStates[i] = state.ModuleStates[i];
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    /**
     * Calls the Logger to publish telemetry data. Call this method from the same thread as
     * CommandScheduler.
     */
    public void logDataSynchronously() {
        Pose2d pose = new Pose2d(latestPose.getX(), latestPose.getY(), latestPose.getRotation());
        Logger.recordOutput("Telemetry/fieldToRobot", pose);
        Logger.recordOutput("Telemetry/moduleStates", moduleStates);
    }

    public double getRotationRadians() {
        return robotRotation;
    }

    public Pose2d getFieldToRobot() {
        return latestPose;
    }

    public Translation2d getVelocity() {
        // return new Translation2d(
        //         velocityFilter.filter(velocityXFieldRelative),
        //         velocityFilter.filter(velocityYFieldRelative));
        return new Translation2d(velocityXFieldRelative, velocityYFieldRelative);
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

    public double getVelocityX() {
        return velocityXFieldRelative;
    }

    public double getVelocityY() {
        return velocityYFieldRelative;
    }
}
