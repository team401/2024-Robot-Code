package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants.CameraParams;
import frc.robot.subsystems.localization.CameraIO.CameraIOInputs;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {

    private VisionSystemSim visionSim = new VisionSystemSim("main");

    private List<CameraIO> cameras = new ArrayList<>();
    private List<CameraIOInputs> inputs = new ArrayList<>();

    private Supplier<Pose2d> getFieldToRobot = () -> new Pose2d();

    public VisionIOSim(List<CameraParams> params) {
        for (CameraParams param : params) {
            cameras.add(CameraIOPhoton.fromSimCameraParams(param, visionSim));
            inputs.add(new CameraIOInputs());
        }
    }

    public List<CameraIOInputs> getInputs() {
        visionSim.update(getFieldToRobot.get());

        for (int i = 0; i < cameras.size(); i++) {
            cameras.get(i).updateInputs(inputs.get(i));
        }

        return inputs;
    }
}
