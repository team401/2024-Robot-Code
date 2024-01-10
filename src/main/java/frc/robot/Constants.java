package frc.robot;

public class Constants {
    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : Mode.SIM;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final class CANDevices {
        public static final int motorID = 1;
    }
}
