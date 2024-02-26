package frc.robot.telemetry;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class TelemetryIOSim implements TelemetryIO {
    private Pose3d pose3d = new Pose3d();
    private Pose2d pose2d = new Pose2d();
    private SwerveModuleState[] moduleStates =
            new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            };
    private SwerveModuleState[] moduleTargets =
            new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            };
    private Pigeon2 pigeon = new Pigeon2(0);
    private Pigeon2SimState pigeonSim = pigeon.getSimState();

    private double[] driveCurrents;
    private double[] driveVoltages;
    private double[] rotationCurrents;
    private double[] rotationVoltages;

    public TelemetryIOSim() {}

    @Override
    public void setRobotPose(Pose3d pose) {
        pose3d = pose;
        pigeonSim.setRawYaw(Units.radiansToDegrees(pose.getRotation().getAngle()));
    }

    @Override
    public void setRobotPose(Pose2d pose) {
        pose2d = pose;
        pigeonSim.setRawYaw(pose.getRotation().getDegrees());
    }

    @Override
    public void setSwerveModuleStates(SwerveModuleState[] moduleStates) {
        this.moduleStates = moduleStates;
    }

    @Override
    public void setSwerveModuleTargets(SwerveModuleState[] moduleTargets) {
        this.moduleTargets = moduleTargets;
    }

    @Override
    public void updateInputs(TelemetryIOInputs inputs) {
        inputs.pose3d = pose3d;
        inputs.pose2d = pose2d;
        inputs.moduleStates = moduleStates;
        inputs.moduleTargets = moduleTargets;
        inputs.accelerationX = pigeon.getAccelerationX().getValueAsDouble();
        inputs.accelerationY = pigeon.getAccelerationY().getValueAsDouble();

        inputs.driveCurrents = this.driveCurrents;
        inputs.driveVoltages = this.driveVoltages;
        inputs.rotationCurrents = this.rotationCurrents;
        inputs.rotationVoltages = this.rotationVoltages;
    }

    @Override
    public void setDriveCurrents(double[] driveCurrents) {
        this.driveCurrents = driveCurrents;
    }

    @Override
    public void setDriveVoltages(double[] driveVoltages) {
        this.driveVoltages = driveVoltages;
    }

    @Override
    public void setRotationCurrents(double[] rotationCurrents) {
        this.rotationCurrents = rotationCurrents;
    }

    @Override
    public void setRotationVoltages(double[] rotationVoltages) {
        this.rotationVoltages = rotationVoltages;
    }
}
