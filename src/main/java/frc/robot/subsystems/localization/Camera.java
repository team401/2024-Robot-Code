package frc.robot.subsystems.localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraParams;
import frc.robot.subsystems.localization.VisionLocalizer.CameraMeasurement;
import frc.robot.utils.AllianceUtil;
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
        // If the robot is not in teleop, trust cameras based on their location relative to the tags
        if (!DriverStation.isTeleop()) {
            switch (zone) {
                case LEFT:
                    if (AllianceUtil.isRightOfSpeaker(inputs.latestFieldToRobot.getY(), 2)) {
                        return VisionConstants.lowCameraUncertainty;
                    }
                    break;
                case RIGHT:
                    if (AllianceUtil.isLeftOfSpeaker(inputs.latestFieldToRobot.getY(), 2)) {
                        return VisionConstants.lowCameraUncertainty;
                    }
                    break;
                case MIDDLE:
                    break;
            }
        }

        // If the robot is very close, trust highly
        if (inputs.averageTagDistanceM < VisionConstants.skewCutoffDistance) {
            return DriverStation.isTeleop()
                    ? VisionConstants.teleopCameraUncertainty
                    : VisionConstants.lowCameraUncertainty;
            // If the robot is somewhat close, check if the cameras are at extreme angles, and trust
            // accordingly
        } else if (inputs.averageTagDistanceM < VisionConstants.lowUncertaintyCutoffDistance) {
            return Math.abs(inputs.averageTagYaw.getDegrees()) < VisionConstants.skewCutoffRotation
                    ? VisionConstants.lowCameraUncertainty
                    : VisionConstants.highCameraUncertainty;
            // If the robot is past the final distance cutoff, distrust
        } else {
            return VisionConstants.highCameraUncertainty;
        }
    }
}
