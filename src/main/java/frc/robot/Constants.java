package frc.robot;

public class Constants {
    public static final double loopTime = 0.02;

    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : Mode.SIM;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final class CANDevices {
        public static final int motorID = 1;
    }

    public static final class Scoring {
        public static final double aimerkP = 1.0;
        public static final double aimerkI = 0.0;
        public static final double aimerkD = 0.0;

        public static final double aimerkS = 0.0;
        public static final double aimerkG = 0.0;
        public static final double aimerkV = 0.0;
        public static final double aimerkA = 0.0;


        public static final double shooterkP = 1.0;
        public static final double shooterkI = 0.0;
        public static final double shooterkD = 0.0;

        public static final double shooterkS = 0.0;
        public static final double shooterkV = 0.0;
        public static final double shooterkA = 0.0;
    }
}
