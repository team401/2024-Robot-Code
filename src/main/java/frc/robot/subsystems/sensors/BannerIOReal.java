package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class BannerIOReal implements BannerIO {
    private final DigitalInput banner;

    public BannerIOReal(int bannerPort) {
        banner = new DigitalInput(bannerPort);
    }

    @Override
    public void updateInputs(BannerIOInputs inputs) {
        inputs.hasNote = banner.get();
    }
}
