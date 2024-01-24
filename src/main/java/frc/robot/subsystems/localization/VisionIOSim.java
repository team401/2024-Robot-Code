package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants.CameraParams;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {

    private VisionSystemSim visionSim = new VisionSystemSim("main");

    private List<CameraIO> cameras = new ArrayList<>();
    private List<CameraIOInputsAutoLogged> inputs = new ArrayList<>();

    private Supplier<Pose2d> getFieldToRobot;

    public VisionIOSim(List<CameraParams> params, Supplier<Pose2d> getFieldToRobot) {
        this.getFieldToRobot = getFieldToRobot;

        for (CameraParams param : params) {
            cameras.add(CameraIOPhoton.fromSimCameraParams(param, visionSim));
            inputs.add(new CameraIOInputsAutoLogged());
        }
    }

    public List<CameraIOInputsAutoLogged> getInputs() {
        visionSim.update(getFieldToRobot.get());

        SmartDashboard.putData("PhotonSimField", visionSim.getDebugField());

        for (int i = 0; i < cameras.size(); i++) {
            cameras.get(i).updateInputs(inputs.get(i));

            Logger.processInputs("Vision/Camera" + i, inputs.get(i));
        }

        return inputs;
    }
}
