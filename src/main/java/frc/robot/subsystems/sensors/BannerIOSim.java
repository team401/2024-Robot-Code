package frc.robot.subsystems.sensors;

public class BannerIOSim implements BannerIO {
    @Override
    public void updateInputs(BannerIOInputs inputs) {
        inputs.hasNote = true;
    }
}
