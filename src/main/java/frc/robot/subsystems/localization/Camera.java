package frc.robot.subsystems.localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraParams;
import frc.robot.subsystems.localization.VisionLocalizer.CameraMeasurement;
import org.littletonrobotics.junction.Logger;

public class Camera {

    public enum CameraTrustZone {
        LEFT,
        RIGHT,
        MIDDLE,
    }

    public final String name;
    public final CameraTrustZone zone;

    private final CameraIO io;
    private final CameraIOInputsAutoLogged inputs;

    public Camera(CameraParams params, CameraIO io) {
        name = params.name();
        zone = params.zone();

        this.io = io;
        inputs = new CameraIOInputsAutoLogged();
    }

    public void update() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision/" + name, inputs);
    }

    public boolean hasNewMeasurement() {
        return inputs.wasAccepted;
    }

    public boolean isConnected() {
        return inputs.connected;
    }

    public CameraMeasurement getLatestMeasurement() {
        return new CameraMeasurement(
                inputs.latestFieldToRobot, inputs.latestTimestampSeconds, getLatestVariance());
    }

    public Matrix<N3, N1> getLatestVariance() {
        if (!DriverStation.isTeleop()) {
            if (inputs.latestFieldToRobot.getY() > FieldConstants.fieldToBlueSpeaker.getY() - 2
                    && zone == CameraTrustZone.LEFT) {
                return VisionConstants.highCameraUncertainty;
            }

            if (inputs.latestFieldToRobot.getY() < FieldConstants.fieldToBlueSpeaker.getY() + 2
                    && zone == CameraTrustZone.RIGHT) {
                return VisionConstants.highCameraUncertainty;
            }
        }

        if (inputs.averageTagDistanceM < VisionConstants.skewCutoffDistance) {
            if (!DriverStation.isTeleop()) {
                return VisionConstants.lowCameraUncertainty;
            } else {
                return VisionConstants.teleopCameraUncertainty;
            }
        } else if (inputs.averageTagDistanceM < VisionConstants.lowUncertaintyCutoffDistance
                && Math.abs(inputs.averageTagYaw.getDegrees())
                        < VisionConstants.skewCutoffRotation) {
            return VisionConstants.lowCameraUncertainty;
        } else {
            return VisionConstants.highCameraUncertainty;
        }
    }
}
