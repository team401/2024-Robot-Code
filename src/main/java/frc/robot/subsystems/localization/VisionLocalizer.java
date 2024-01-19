package frc.robot.subsystems.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionLocalizer extends SubsystemBase {
    private final List<CameraIO> cameras;
    private final List<CameraIOInputsAutoLogged> cameraInputs = new ArrayList<>();

    // avoid NullPointerExceptions by setting a default no-op
    private Consumer<CameraMeasurement> cameraConsumer = (c) -> {};

    private Supplier<Pose2d> fieldToRobotSupplier;

    public VisionLocalizer(List<CameraIO> cameras) {
        this.cameras = cameras;
        for (int i = 0; i < cameras.size(); i++) {
            cameraInputs.add(new CameraIOInputsAutoLogged());
        }
    }


    @Override
    public void periodic() {
        for (int i = 0; i < cameras.size(); i++) {
            cameras.get(i).updateInputs(cameraInputs.get(i));
            
            var inputs = cameraInputs.get(i);

            cameraConsumer.accept(
                new CameraMeasurement(
                    inputs.latestFieldToRobot,
                    inputs.latestTimestampSeconds,
                    cameraUncertainty(inputs.averageTagDistanceM)));

            Logger.recordOutput("Vision/"+cameras.get(i).getName()+"/fieldToRobot", inputs.latestFieldToRobot);
        }
    }

    public void setCameraConsumer(Consumer<CameraMeasurement> cameraConsumer) {
        this.cameraConsumer = cameraConsumer;
    }

    private Matrix<N3, N1> cameraUncertainty(double averageTagDistanceM) {
        /*
         * On this year's field, Apriltags are arranged into rough 'corridors' between the stage and
         * speaker, and a central 'desert,' where few tags can be found. It follows that we should
         * determine the variance of our camera mesurements based on that.
         */
        if (averageTagDistanceM < 2.0 && this.robotInMidField()) {
            return VisionConstants.lowCameraUncertainty;
        } else {
            return VisionConstants.highCameraUncertainty;
        }
    }

    private boolean robotInMidField() {
        return fieldToRobotSupplier.get().getX() > VisionConstants.midfieldLowThreshold
            && fieldToRobotSupplier.get().getX() < VisionConstants.midfieldHighThreshold;
    }

    /**
     * This class exists solely because java has no functional interface for a function with 3 inputs
     */
    public static record CameraMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> variance) {}
}
