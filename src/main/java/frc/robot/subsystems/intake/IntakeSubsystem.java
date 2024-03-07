package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.sensors.SensorIO;
import frc.robot.subsystems.sensors.SensorIOInputsAutoLogged;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private State state = State.IDLE;

    private BooleanSupplier scorerWantsNote = () -> false;

    private final SensorIO sensorIo;
    private final SensorIOInputsAutoLogged sensorInputs = new SensorIOInputsAutoLogged();

    private IntakeAction action = IntakeAction.NONE;

    private double intakeOverrideVolts = 0.0;
    private double beltOverrideVolts = 0.0;

    public IntakeSubsystem(IntakeIO io, SensorIO sensorIo) {
        this.io = io;
        this.sensorIo = sensorIo;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        sensorIo.updateInputs(sensorInputs);
        Logger.processInputs("intake", inputs);
        Logger.processInputs("sensors", sensorInputs);

        switch (state) {
            case IDLE:
                idle();
                break;
            case SEEKING:
                seeking();
                break;
            case REVERSING:
                reversing();
                break;
            case OVERRIDE:
                override();
                break;
        }

        Logger.recordOutput("intake/running", inputs.leftIntakeVoltage != 0.0);
        Logger.recordOutput("intake/belting", inputs.beltVoltage != 0.0);

        Logger.recordOutput("intake/state", state.toString());
    }

    public void setScoringSupplier(BooleanSupplier scorerWantsNote) {
        this.scorerWantsNote = scorerWantsNote;
    }

    public void run(IntakeAction action) {
        this.action = action;
    }

    public void toggle() {
        if (action == IntakeAction.NONE) {
            action = IntakeAction.INTAKE;
        } else {
            action = IntakeAction.NONE;
        }
    }

    private void idle() {
        if (action == IntakeAction.INTAKE) {
            state = State.SEEKING;
        } else if (action == IntakeAction.REVERSE) {
            state = State.REVERSING;
        } else if (action == IntakeAction.OVERRIDE) {
            state = State.OVERRIDE;
        }

        io.setBeltVoltage(0);
        io.setIntakeVoltage(0);
    }

    private void seeking() {
        if (action != IntakeAction.INTAKE) {
            state = State.IDLE;
        }

        io.setIntakeVoltage(IntakeConstants.intakePower);
        io.setBeltVoltage(IntakeConstants.beltPower);
    }

    private void reversing() {
        if (action != IntakeAction.REVERSE) {
            state = State.IDLE;
        }

        io.setIntakeVoltage(-IntakeConstants.intakePower);
        io.setBeltVoltage(-IntakeConstants.beltPower);
    }

    private void override() {
        if (action != IntakeAction.OVERRIDE) {
            state = State.IDLE;
        }

        io.setIntakeVoltage(intakeOverrideVolts);
        io.setBeltVoltage(beltOverrideVolts);
    }

    public void setOverrideVolts(double intake, double belt) {
        intakeOverrideVolts = intake;
        beltOverrideVolts = belt;
    }

    private enum State {
        IDLE, // do nothing
        SEEKING, // run intake wheels until a note is taken in
        REVERSING, // whole intake backwards
        OVERRIDE
    }

    public enum IntakeAction {
        NONE, // do nothing
        INTAKE, // Try to intake a note if you don't have one
        REVERSE, // run backwards
        OVERRIDE
    }
}
