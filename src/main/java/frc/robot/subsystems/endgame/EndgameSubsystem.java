package frc.robot.subsystems.endgame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndgameConstants;
import org.littletonrobotics.junction.Logger;

public class EndgameSubsystem extends SubsystemBase {
    private PIDController endgameController = new PIDController(0, 0, 0);
    private double endgamekP = 0.03;
    private double endgamekI = 0.0;
    private double endgamekD = 0.0;
    EndgameIO endgameIO;
    EndgameIOInputsAutoLogged endgameInputs = new EndgameIOInputsAutoLogged();

    // SmartDashboard
    // SmartDashboard
    private double leftEndgameMotorPower = 0.5;
    private double rightEndgameMotorPower = 0.5;
    private double endgameGoalPosition;

    public EndgameSubsystem(EndgameIO endgameIO) {
        this.endgameIO = endgameIO;
    }

    public void setEndgamePosition() {
        // endgamekP = SmartDashboard.getNumber("endgame kP", 0.03);
        // endgamekI = SmartDashboard.getNumber("endgame kI", 0.0);
        // endgamekD = SmartDashboard.getNumber("endgame kD", 0.0);

    }

    public void endgameGoUp() {
        endgameGoalPosition = EndgameConstants.endgameUp;
    }

    public void endgameGoDown() {
        endgameGoalPosition = EndgameConstants.endgameDown;
    }

    public void endgameControl() {
        double leftOutput =
                endgameController.calculate(endgameInputs.encoderLeftPosition, endgameGoalPosition);
        double rightOutput =
                endgameController.calculate(
                        endgameInputs.encoderRightPosition, endgameGoalPosition);
        endgameIO.setEndgameMotorPower(leftOutput, rightOutput);
        double amps = endgameInputs.endgameAmps;
        SmartDashboard.putNumber("left endgame motor power", leftOutput);
        SmartDashboard.putNumber("right endgame motor power", rightOutput);
    }

    @Override
    public void periodic() {
        endgameIO.updateInputs(endgameInputs);
        endgameController.setPID(endgamekP, endgamekI, endgamekD);
        endgameControl();

        Logger.recordOutput(
                "endgame/Elevator3d",
                new Pose3d(
                        0.0,
                        0.0,
                        endgameInputs.encoderLeftPosition + 0.1,
                        new Rotation3d(0, 0, 0)));
    }
}
