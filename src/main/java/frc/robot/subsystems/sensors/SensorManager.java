package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorManager extends SubsystemBase {
    private final BannerIO bannerIO;
    private final BannerIOInputsAutoLogged bannerInputs = new BannerIOInputsAutoLogged();

    public SensorManager(BannerIO bannerIO) {
        this.bannerIO = bannerIO;
    }

    public boolean hasNote() {
        return bannerInputs.hasNote;
    }

    @Override
    public void periodic() {
        bannerIO.updateInputs(bannerInputs);
    }
}
