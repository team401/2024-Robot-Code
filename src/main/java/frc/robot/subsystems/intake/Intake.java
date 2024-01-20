package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private State state = State.IDLE;

    private boolean shouldBeRunning = false;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        switch (state) {
            case IDLE:
                idle();
                break;
            case SEEKING:
                seeking();
                break;
            case FEEDING:
                feeding();
                break;
        }

        Logger.recordOutput("Intake/running", inputs.backMotorVoltage != 0.0);
    }

    public void run(boolean shouldBeRunning) {
        this.shouldBeRunning = shouldBeRunning;
    }

    public boolean hasNote() {
        return state == State.FEEDING;
    }

    private void idle() {
        if (shouldBeRunning) {
            // TODO: if the shooter has a note, don't intake
            state = State.SEEKING;
            io.setIntakeVoltage(5);
        }
    }

    private void seeking() {
        if (inputs.backMotorCurrent > 20) {
            state = State.FEEDING;
            io.setIntakeVoltage(2);
        }
    }

    private void feeding() {
        if (inputs.backMotorCurrent < 5) {
            state = State.IDLE;
            io.setIntakeVoltage(0);
        }
    }

    private enum State {
        IDLE,
        SEEKING,
        FEEDING
    }
}
