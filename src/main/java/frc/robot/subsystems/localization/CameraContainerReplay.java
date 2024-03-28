package frc.robot.subsystems.localization;

import frc.robot.Constants.VisionConstants.CameraParams;
import java.util.ArrayList;
import java.util.List;

public class CameraContainerReplay implements CameraContainer {
    private List<Camera> cameras = new ArrayList<>();

    public CameraContainerReplay(List<CameraParams> params) {
        for (CameraParams param : params) {
            cameras.add(new Camera(param, new CameraIO() {}));
        }
    }

    public List<Camera> getCameras() {
        return cameras;
    }

    public void update() {
        for (Camera camera : cameras) {
            camera.update();
        }
    }
}
