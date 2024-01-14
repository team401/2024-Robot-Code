package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.EndgameConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EndgameSubsystem;

public class EndgameSubsystem extends SubsystemBase{
    private PIDController endgameController = new PIDController(0, 0, 0);
    private double endgamekP = 0.03;
    private double endgamekI = 0.0;
    private double endgamekD = 0.0;

    // SmartDashboard 
    private double leftEndgameMotorPower = 0.5;
    private double rightEndgameMotorPower = 0.5;
    private CANSparkMax leftEndgameMotor, rightEndgameMotor;
    private final double CURRENT_LIMIT = 25.0;
    private double endgameGoalPosition;

    public EndgameSubsystem() {
    leftEndgameMotor = new CANSparkMax(EndgameConstants.leftMotorID, MotorType.kBrushless);
    rightEndgameMotor = new CANSparkMax(EndgameConstants.rightMotorID, MotorType.kBrushless);
    /*leftMotor.follow(rightMotor, true);*/
    leftEndgameMotor.setSmartCurrentLimit(80);
    rightEndgameMotor.setSmartCurrentLimit(80);

    rightEndgameMotor.setIdleMode(IdleMode.kBrake);
    leftEndgameMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setEndgamePosition() {
    endgamekP = SmartDashboard.getNumber("endgame kP", 0.03);
    endgamekI = SmartDashboard.getNumber("endgame kI", 0.0);
    endgamekD = SmartDashboard.getNumber("endgame kD", 0.0);
  }

  public void setEndgameMotorPower(double percent) {
    rightEndgameMotor.set(-percent);
  }

  public double getEndgameMotorAmps() {
    return rightEndgameMotor.getOutputCurrent();
  }

  public double getEndgamePosition(){
    return rightEndgameMotor.getEncoder().getPosition();
  }

  private void checkEndgameAmps() {
    if (getEndgameMotorAmps() > CURRENT_LIMIT) {
      setEndgameMotorPower(0);
    }
  }

  public void endgameGoUp(){
    endgameGoalPosition = EndgameConstants.endgameUp;
  }
  public void endgameGoDown(){
    endgameGoalPosition = EndgameConstants.endgameDown;
  }

  public void wristControl() {
    double output = endgameController.calculate(getEndgamePosition(), endgameGoalPosition);
    setEndgameMotorPower(output);
    checkEndgameAmps();
}

  @Override
  public void periodic(){
    endgameController.setPID(endgamekP, endgamekI, endgamekD);
    SmartDashboard.putNumber("left endgame motor power", leftEndgameMotorPower);
    SmartDashboard.putNumber("right endgame motor power", rightEndgameMotorPower);
  }
}