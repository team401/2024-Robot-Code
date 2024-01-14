package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.EndgameConstants;


public class SimIO extends IO {
    Encoder leftEndgameEncoder = new Encoder (0, 1);
    Encoder rightEndgameEncoder = new Encoder (2,3);
    private CANSparkMax leftEndgameMotor, rightEndgameMotor;
  
    public void resetEncoders(){
        leftEndgameEncoder.reset();
        rightEndgameEncoder.reset();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("endgame left encoder distance", leftEndgameMotor.getEncoder().getPosition()/EndgameConstants.ticksPerFoot);
        SmartDashboard.putNumber("endgame right encoder distance", rightEndgameMotor.getEncoder().getPosition()/EndgameConstants.ticksPerFoot);
    }
}
