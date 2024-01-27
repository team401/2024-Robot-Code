package frc.robot.subsystems.endgame;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.EndgameConstants;

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
        rightEndgameMotor.set(-rightPercent);
        leftEndgameMotor.set(-leftPercent);
    }

    public double getRightEndgameMotorAmps() {
        return rightEndgameMotor.getOutputCurrent();
    }

    public double getLeftEndgameMotorAmps() {
        return leftEndgameMotor.getOutputCurrent();
    }

    public double getRightEndgamePosition() {
        return rightEndgameMotor.getEncoder().getPosition() / EndgameConstants.ticksPerFoot;
    }

    public double getLeftEndgamePosition() {
        return leftEndgameMotor.getEncoder().getPosition() / EndgameConstants.ticksPerFoot;
    }

    public void checkEndgameAmps() {
        if (getLeftEndgameMotorAmps() > CURRENT_LIMIT
                || getRightEndgameMotorAmps() > CURRENT_LIMIT) {
            endgameIOinputs.endgameLeftMotorCurrent = 0;
            endgameIOinputs.endgameRightMotorCurrent = 0;
        }
    }
}
