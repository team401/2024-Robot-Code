package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {

    @AutoLog
    public static class CameraIOInputs {
        public Pose2d latestFieldToRobot;
        public double latestTimestampSeconds;
        public double averageTagDistanceM;
        public String name;
    }

    public default void updateInputs(CameraIOInputs inputs) {}
}
