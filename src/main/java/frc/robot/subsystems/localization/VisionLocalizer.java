package frc.robot.subsystems.localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.localization.CameraIO.CameraIOInputs;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionLocalizer extends SubsystemBase {

    private VisionIO io;

    // avoid NullPointerExceptions by setting a default no-op
    private Consumer<CameraMeasurement> cameraConsumer = (c) -> {};

    private Supplier<Pose2d> fieldToRobotSupplier = () -> new Pose2d();

    public VisionLocalizer(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        for (CameraIOInputs inputs : io.getInputs()) {
            cameraConsumer.accept(
                    new CameraMeasurement(
                            inputs.latestFieldToRobot,
                            inputs.latestTimestampSeconds,
                            cameraUncertainty(inputs.averageTagDistanceM, inputs.nTags)));
        }

        Logger.recordOutput("Vision/robotInMidField", robotInMidField());
    }

    public void setCameraConsumer(Consumer<CameraMeasurement> cameraConsumer) {
        this.cameraConsumer = cameraConsumer;
    }

    public void setFieldToRobotSupplier(Supplier<Pose2d> fieldToRobotSupplier) {
        this.fieldToRobotSupplier = fieldToRobotSupplier;
    }

    private Matrix<N3, N1> cameraUncertainty(double averageTagDistanceM, int nTags) {
        /*
         * On this year's field, AprilTags are arranged into rough 'corridors' between the stage and
         * speaker, and a central 'desert,' where few tags can be found. It follows that we should
         * determine the variance of our camera measurements based on that.
         */
        if (nTags < 2) {
            return VisionConstants.singleTagUncertainty;
        } else if (averageTagDistanceM < 2.0 && this.robotInMidField()) {
            return VisionConstants.lowCameraUncertainty;
        } else {
            return VisionConstants.highCameraUncertainty;
        }
    }

    private boolean robotInMidField() {
        return fieldToRobotSupplier.get().getX() > FieldConstants.midfieldLowThresholdM
                && fieldToRobotSupplier.get().getX() < FieldConstants.midfieldHighThresholdM;
    }

    /**
     * This class exists solely because java has no functional interface for a function with 3
     * inputs
     */
    public static record CameraMeasurement(
            Pose2d pose, double timestamp, Matrix<N3, N1> variance) {}
}
