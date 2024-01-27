package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {

    @AutoLog
    public static class CameraIOInputs {
        public Pose2d latestFieldToRobot = new Pose2d();
        public double averageTagDistanceM = 0.0;
        public int nTags = 0;

        public double latestTimestampSeconds = 0.0;
        public boolean connected = false;
        public boolean isNew = false;
    }

    public default void updateInputs(CameraIOInputs inputs) {}
}
