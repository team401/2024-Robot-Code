package frc.robot.subsystems.localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.localization.CameraIO.CameraIOInputs;
import java.util.function.Consumer;

public class VisionLocalizer extends SubsystemBase {

    private CameraContainer io;

    // avoid NullPointerExceptions by setting a default no-op
    private Consumer<CameraMeasurement> cameraConsumer = (c) -> {};

    public VisionLocalizer(CameraContainer io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        int i = 0;
        for (CameraIOInputs inputs : io.getInputs()) {
            cameraConsumer.accept(
                    new CameraMeasurement(
                            inputs.latestFieldToRobot,
                            inputs.latestTimestampSeconds,
                            cameraUncertainty(inputs.averageTagDistanceM)));

            SmartDashboard.putBoolean("Camera " + i + " Connected", inputs.connected);
            i++;
        }
    }

    public void setCameraConsumer(Consumer<CameraMeasurement> cameraConsumer) {
        this.cameraConsumer = cameraConsumer;
    }

    private Matrix<N3, N1> cameraUncertainty(double averageTagDistanceM) {
        if (averageTagDistanceM < VisionConstants.lowUncertaintyCutoffDistance) {
            return VisionConstants.lowCameraUncertainty;
        } else {
            return VisionConstants.highCameraUncertainty;
        }
    }

    /**
     * This class exists solely because java has no functional interface for a function with 3
     * inputs
     */
    public static record CameraMeasurement(
            Pose2d pose, double timestamp, Matrix<N3, N1> variance) {}
}
