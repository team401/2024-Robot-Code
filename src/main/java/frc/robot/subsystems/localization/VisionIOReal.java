package frc.robot.subsystems.localization;

import frc.robot.Constants.VisionConstants.CameraParams;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class VisionIOReal implements VisionIO {

    private List<CameraIOPhoton> cameras = new ArrayList<>();
    private List<CameraIOInputsAutoLogged> inputs;

    public VisionIOReal(List<CameraParams> params) {
        for (CameraParams param : params) {
            cameras.add(CameraIOPhoton.fromRealCameraParams(param));
            inputs.add(new CameraIOInputsAutoLogged());
        }
    }

    public List<CameraIOInputsAutoLogged> getInputs() {
        for (int i = 0; i < cameras.size(); i++) {
            cameras.get(i).updateInputs(inputs.get(i));

            Logger.processInputs("Vision/Camera" + i, inputs.get(i));
        }

        return inputs;
    }
}
