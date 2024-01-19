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

    // this value can't be updated by the outside world, so it doesn't count as an input
    public default String getName() { return ""; }
}
