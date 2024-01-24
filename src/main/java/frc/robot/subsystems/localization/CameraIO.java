package frc.robot.subsystems.localization;

import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {

    @AutoLog
    public static class CameraIOInputs {
        // not using a Pose2d due to instability in WPIlib
        public double latestX = 0.0;
        public double latestY = 0.0;
        public double latestTheta = 0.0;

        public double latestTimestampSeconds = 0.0;
        public double averageTagDistanceM = 0.0;
        public boolean connected = false;
    }

    public default void updateInputs(CameraIOInputs inputs) {}
}
