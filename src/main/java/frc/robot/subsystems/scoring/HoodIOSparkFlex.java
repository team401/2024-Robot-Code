// FIXME: Fix once we get the real robot
package frc.robot.subsystems.scoring;

import com.revrobotics.CANSparkFlex;
import frc.robot.Constants.ScoringConstants;

public class HoodIOSparkFlex implements HoodIO {
    private final CANSparkFlex hoodMotor =
            new CANSparkFlex(ScoringConstants.hoodId, CANSparkFlex.MotorType.kBrushless);

    double goalAngleRad = 0.0;

    public HoodIOSparkFlex() {
        hoodMotor.setSmartCurrentLimit(10);

        hoodMotor.getPIDController().setP(ScoringConstants.hoodkP);
        hoodMotor.getPIDController().setI(ScoringConstants.hoodkI);
        hoodMotor.getPIDController().setD(ScoringConstants.hoodkD);

        hoodMotor.getEncoder().setPosition(0);

        // TODO: Get position later
        // hoodMotor.getEncoder().setPositionConversionFactor()
    }

    @Override
    public void setHoodAngleRad(double angle) {
        goalAngleRad = angle;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodAngleRad = hoodMotor.getEncoder().getPosition();
        inputs.hoodGoalAngleRad = goalAngleRad;

        inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput();
        inputs.hoodCurrentAmps = hoodMotor.getOutputCurrent();
    }
}
