package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface BannerIO {
    @AutoLog
    public static class BannerIOInputs {
        public boolean hasNote = false;
    }

    public default void updateInputs(BannerIOInputs inputs) {}
}
