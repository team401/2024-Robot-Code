package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class TelemetryIOLive implements TelemetryIO {
    private Pose3d pose3d = new Pose3d();
    private Pose2d pose2d = new Pose2d();
    private SwerveModuleState[] moduleStates =
            new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            };

    private double rotation;
    private double rotationVelocity;

    // private Pigeon2 pigeon = new Pigeon2(0);

    public TelemetryIOLive() {}

    @Override
    public void setRobotPose(Pose3d pose) {
        pose3d = pose;
    }

    @Override
    public void setRobotPose(Pose2d pose) {
        pose2d = pose;
    }

    @Override
    public void setSwerveModuleStates(SwerveModuleState[] moduleStates) {
        this.moduleStates = moduleStates;
    }

    @Override
    public void setRobotRotation(double rotation) {
        this.rotation = rotation;
    }

    @Override
    public void setRobotRotationVelocity(double rotationVelocity) {
        this.rotationVelocity = rotationVelocity;
    }

    @Override
    public void updateInputs(TelemetryIOInputs inputs) {
        inputs.pose3d = pose3d;
        inputs.pose2d = pose2d;
        inputs.moduleStates = moduleStates;

        inputs.robotRotation = rotation;
        inputs.robotRotationVelocity = rotationVelocity;

        // inputs.accelerationX = pigeon.getAccelerationX().getValueAsDouble();
        // inputs.accelerationY = pigeon.getAccelerationY().getValueAsDouble();
    }
}
