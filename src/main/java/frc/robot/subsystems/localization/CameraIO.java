package frc.robot.subsystems.localization;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface CameraIO {
    
    @AutoLog
    public static class CameraIOInputs {
        public Pose2d latestFieldToRobot;
        public double latestTimestampSeconds;
        public double averageTagDistanceM;
    }

    public default void updateInputs(CameraIOInputs inputs) {}
}
