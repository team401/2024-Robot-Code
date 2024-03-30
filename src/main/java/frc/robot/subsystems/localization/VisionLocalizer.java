package frc.robot.subsystems.localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Consumer;

public class VisionLocalizer extends SubsystemBase {

    private CameraContainer container;

    // avoid NullPointerExceptions by setting a default no-op
    private Consumer<CameraMeasurement> cameraConsumer = (c) -> {};

    public VisionLocalizer(CameraContainer container) {
        this.container = container;
    }

    @Override
    public void periodic() {
        container.update();
        for (Camera camera : container.getCameras()) {
            if (camera.hasNewMeasurement()) {
                cameraConsumer.accept(camera.getLatestMeasurement());
            }
        }
    }

    public void setCameraConsumer(Consumer<CameraMeasurement> cameraConsumer) {
        this.cameraConsumer = cameraConsumer;
    }

    public boolean coprocessorConnected() {
        return container.getCameras().get(0).isConnected();
    }

    /**
     * This class exists solely because java has no functional interface for a function with 3
     * inputs
     */
    public static record CameraMeasurement(
            Pose2d pose, double timestamp, Matrix<N3, N1> variance) {}
}
