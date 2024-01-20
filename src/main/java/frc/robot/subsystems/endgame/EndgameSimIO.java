package frc.robot.subsystems.endgame;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.EndgameConstants;


public class EndgameSimIO implements EndgameIO {
    private CANSparkMax leftEndgameMotor, rightEndgameMotor;
    private final double CURRENT_LIMIT = 25.0;
    private int leftEndgameEncoder;
    private int encoderIndex;
  
    public void EncoderSim​(Encoder encoder){
        encoderIndex = encoder.getFPGAIndex();
    }

    public void updateInputs (EndgameIOInputs inputs) {

        double endgameLeftMotorCurrent = leftEndgameMotor.getLeftEndgameMotorAmps();
        double endgameRightMotorCurrent = rightEndgameMotor.getRightEndgameMotorAmps();
        double encoderLeftPosition = leftEndgameMotor.getLeftEndgamePosition();
        double encoderRightPosition = rightEndgameMotor.getRightEndgamePosition();

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

    public double getRightEndgamePosition(){
        return rightEndgameMotor.getEncoder().getPosition()/EndgameConstants.ticksPerFoot;
    }

    public double getLeftEndgamePosition(){
        return leftEndgameMotor.getEncoder().getPosition()/EndgameConstants.ticksPerFoot;
    }

    public void checkEndgameAmps() {
        if (getLeftEndgameMotorAmps() > CURRENT_LIMIT || getRightEndgameMotorAmps() > CURRENT_LIMIT) {
        endgameIO.setEndgameMotorPower(0, 0);
        }
    }
}
