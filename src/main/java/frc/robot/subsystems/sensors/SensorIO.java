package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {
    @AutoLog
    public static class SensorIOInputs {
        public boolean noteInShootDeck = false;
        public boolean noteInUptake = false;
    }

    public default void updateInputs(SensorIOInputs inputs) {}
    ;
}
