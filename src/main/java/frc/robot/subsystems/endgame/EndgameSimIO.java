package frc.robot.subsystems.endgame;

import com.revrobotics.CANSparkMax;

public class EndgameSimIO implements EndgameIO {
    private CANSparkMax leftEndgameMotor, rightEndgameMotor;
    private final double CURRENT_LIMIT = 25.0;
    private int leftEndgameEncoder;
    private int encoderIndex;
    EndgameIOInputs endgameIOinputs = new EndgameIOInputs();

    // public void EncoderSim​(Encoder encoder){
    //  encoderIndex = encoder.getFPGAIndex();
    // }

    public void updateInputs(EndgameIOInputsAutoLogged inputs) {

        endgameIOinputs.endgameLeftMotorCurrent = getLeftEndgameMotorAmps();
        endgameIOinputs.endgameRightMotorCurrent = getRightEndgameMotorAmps();
        endgameIOinputs.encoderLeftPosition = getLeftEndgamePosition();
        endgameIOinputs.encoderRightPosition = getRightEndgameMotorAmps();
    }

    public void setEndgameMotorPower(double leftPercent, double rightPercent) {
        // no-op to avoid a crash until an ElevatorSim is implemented

        // rightEndgameMotor.set(-rightPercent);
        // leftEndgameMotor.set(-leftPercent);
    }

    public double getRightEndgameMotorAmps() {
        // no-op to avoid a crash until an ElevatorSim is implemented

        // return rightEndgameMotor.getOutputCurrent();
        return 0;
    }

    public double getLeftEndgameMotorAmps() {
        // return leftEndgameMotor.getOutputCurrent();
        // no-op to avoid a crash until an ElevatorSim is implemented
        return 0;
    }

    public double getRightEndgamePosition() {
        // return rightEndgameMotor.getEncoder().getPosition() / EndgameConstants.ticksPerFoot;
        // no-op to avoid a crash until an ElevatorSim is implemented
        return 0;
    }

    public double getLeftEndgamePosition() {
        // return leftEndgameMotor.getEncoder().getPosition() / EndgameConstants.ticksPerFoot;
        // no-op to avoid a crash until an ElevatorSim is implemented
        return 0;
    }

    public void checkEndgameAmps() {
        if (getLeftEndgameMotorAmps() > CURRENT_LIMIT
                || getRightEndgameMotorAmps() > CURRENT_LIMIT) {
            endgameIOinputs.endgameLeftMotorCurrent = 0;
            endgameIOinputs.endgameRightMotorCurrent = 0;
        }
    }
}
