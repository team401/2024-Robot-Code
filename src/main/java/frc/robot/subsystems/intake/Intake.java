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
            case HOLDING:
                holding();
                break;
            case PASSING:
                passing();
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
            state = State.SEEKING;
            io.setIntakeVoltage(5);
        }
    }

    private void seeking() {
        if (inputs.backMotorCurrent > 20) {
            state = State.FEEDING;
            io.setIntakeVoltage(2);
            io.setBeltVolatge(2);
        }
    }

    private void feeding() {
        if (inputs.backMotorCurrent < 5) {
            state = State.HOLDING;
            io.setIntakeVoltage(0);
            io.setBeltVolatge(0);
        }
    }

    private void holding() {
        // TODO: interact with shooter
        /* 
         * if the shooter is ready to take a note, start the belt and transition
         * to passing.
         */
    }

    private void passing() {
        // TODO: interact with shooter
        /*
         * if the shooter has the note (banner sensor), stop the belt and
         * return to idle.
         */
    }

    private enum State {
        IDLE, // do nothing
        SEEKING, // running intake wheels in search of a note
        FEEDING, // taking in a note and passing it to the belt
        HOLDING, // holding a note in the belt until the shooter can take it
        PASSING // moving the belt to pass a note to the shooter
    }
}
