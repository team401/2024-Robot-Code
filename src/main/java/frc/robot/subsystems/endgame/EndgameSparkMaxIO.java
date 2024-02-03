package frc.robot.subsystems.endgame;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.EndgameConstants;

public class EndgameSparkMaxIO implements EndgameIO {

    EndgameIOInputs endgameIOinputs = new EndgameIOInputs();
    CANSparkMax leftEndgameMotor =
            new CANSparkMax(EndgameConstants.leftMotorID, MotorType.kBrushless);
    CANSparkMax rightEndgameMotor =
            new CANSparkMax(EndgameConstants.rightMotorID, MotorType.kBrushless);
    /*leftMotor.follow(rightMotor, true);*/
    final double CURRENT_LIMIT = 25.0;

    public void updateInputs(EndgameIOInputsAutoLogged inputs) {

        endgameIOinputs.endgameLeftMotorCurrent = getLeftEndgameMotorAmps();
        endgameIOinputs.endgameRightMotorCurrent = getRightEndgameMotorAmps();
        endgameIOinputs.encoderLeftPosition = getLeftEndgamePosition();
        endgameIOinputs.encoderRightPosition = getRightEndgameMotorAmps();
    }

    public EndgameSparkMaxIO() {

        leftEndgameMotor.setSmartCurrentLimit(80);
        rightEndgameMotor.setSmartCurrentLimit(80);

        rightEndgameMotor.setIdleMode(IdleMode.kBrake);
        leftEndgameMotor.setIdleMode(IdleMode.kBrake);
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