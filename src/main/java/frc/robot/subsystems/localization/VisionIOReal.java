package frc.robot.subsystems.localization;

import frc.robot.Constants.VisionConstants.CameraParams;
import frc.robot.subsystems.localization.CameraIO.CameraIOInputs;
import java.util.ArrayList;
import java.util.List;

public class VisionIOReal implements VisionIO {

    private List<CameraIOPhoton> cameras = new ArrayList<>();
    private List<CameraIOInputs> inputs;

    public VisionIOReal(List<CameraParams> params) {
        for (CameraParams param : params) {
            cameras.add(CameraIOPhoton.fromRealCameraParams(param));
            inputs.add(new CameraIOInputsAutoLogged());
        }
    }

    public List<CameraIOInputs> getInputs() {
        for (int i = 0; i < cameras.size(); i++) {
            cameras.get(i).updateInputs(inputs.get(i));
        }

        return inputs;
    }
}
