package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.SensorConstants;

public class SensorIORealAndSim implements SensorIO {
    private DigitalInput shooterSensor;
    private DigitalInput uptakeSensor;

    public SensorIORealAndSim() {
        shooterSensor = new DigitalInput(SensorConstants.bannerSensorPort);
        uptakeSensor = new DigitalInput(SensorConstants.uptakeSensorPort);
    }

    @Override
    public void updateInputs(SensorIOInputs inputs) {
        inputs.noteInShootDeck = !shooterSensor.get();
        inputs.noteInUptake = !uptakeSensor.get();
    }
}
