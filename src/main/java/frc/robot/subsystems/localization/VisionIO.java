package frc.robot.subsystems.localization;

import frc.robot.subsystems.localization.CameraIO.CameraIOInputs;
import java.util.List;

/** This is not an AdvantageKit IO interface, but a class to hold cameras and associated data */
public interface VisionIO {

    public List<CameraIOInputs> getInputs();
}
