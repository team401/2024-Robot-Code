package frc.robot.subsystems.endgame;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.EndgameConstants;
import frc.robot.subsystems.endgame.EndgameSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndgameSparkMaxIO implements EndgameIO{

    

    public EndgameSparkMaxIO(){

    CANSparkMax leftEndgameMotor = new CANSparkMax(EndgameConstants.leftMotorID, MotorType.kBrushless);
    CANSparkMax rightEndgameMotor = new CANSparkMax(EndgameConstants.rightMotorID, MotorType.kBrushless);
    /*leftMotor.follow(rightMotor, true);*/
    leftEndgameMotor.setSmartCurrentLimit(80);
    rightEndgameMotor.setSmartCurrentLimit(80);

    rightEndgameMotor.setIdleMode(IdleMode.kBrake);
    leftEndgameMotor.setIdleMode(IdleMode.kBrake);
    }
}
