package frc.robot.subsystems.localization;

import java.util.List;

/** This is not an AdvantageKit IO interface, but a class to hold cameras and associated data */
public interface CameraContainer {

    public List<Camera> getCameras();

    public void update();
}
