package frc.robot.subsystems.localization;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class CameraContainerReplay implements CameraContainer {
    private List<CameraIO> cameras = new ArrayList<>();
    private List<CameraIOInputsAutoLogged> inputs;

    public CameraContainerReplay(int nCameras) {
        for (int i = 0; i < nCameras; i++) {
            cameras.add(new CameraIO() {});
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
