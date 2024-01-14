package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class ScoringSubsystem extends SubsystemBase {
    private final ShooterIO shooterIo;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final AimerIO aimerIo;
    private final AimerIOInputsAutoLogged aimerInputs = new AimerIOInputsAutoLogged();

    private final Timer timer = new Timer();

    private enum ScoringState {
        WAIT,
        INTAKE,
        PRIME,
        SHOOT,
        ENDGAME
    }

    private ScoringState state = ScoringState.WAIT;

    public ScoringSubsystem(ShooterIO shooterIo, AimerIO aimerIo) {
        this.shooterIo = shooterIo;
        this.aimerIo = aimerIo;
    }

    public void intake() {
        if (state == ScoringState.WAIT) {
            enterIntake();
        }
    }

    public void aim() {
        if (state == ScoringState.WAIT && true) { // TODO: Add banner sensor
            enterPrime();
        }
    }

    public void shoot() {
        if (state == ScoringState.PRIME) {
            enterShoot();
        }
    }

    public void abort() {
        if (state == ScoringState.PRIME || state == ScoringState.SHOOT) {
            enterWait();
        }
    }

    public void endgame() {
        if (state == ScoringState.WAIT) {
            enterEndgame();
        }
    }

    private void enterWait() {
        state = ScoringState.WAIT;

        aimerIo.setAimAngRad(0);
        shooterIo.setShooterVelRPM(0);
        shooterIo.setKickerVolts(0);
    }

    private void enterIntake() {
        state = ScoringState.INTAKE;

        aimerIo.setAimAngRad(0);
        shooterIo.setShooterVelRPM(-10);
        shooterIo.setKickerVolts(-1);
    }

    private void enterPrime() {
        state = ScoringState.PRIME;

        shooterIo.setShooterVelRPM(100);
    }

    private void enterShoot() {
        state = ScoringState.SHOOT;

        shooterIo.setKickerVolts(1);
        timer.reset();
        timer.start();
    }

    private void enterEndgame() {
        state = ScoringState.ENDGAME;
    }

    private double findShootAngleRads() { // TODO: Interpolate
        double distancetoGoal = Math.sqrt(Math.pow(Math.sqrt(Math.pow(1, 2) + Math.pow(1, 2)), 2) + Math.pow(1, 2));
        return Math.atan2(distancetoGoal, 1);
    }

    @Override
    public void periodic() {
        shooterIo.updateInputs(shooterInputs);
        aimerIo.updateInputs(aimerInputs);

        switch (state) {
            case WAIT:
                break;
            case INTAKE:
                if (true) { // TODO: Banner sensor triggered
                    enterWait();
                }
                break;
            case PRIME:
                aimerIo.setAimAngRad(findShootAngleRads());
                break;
            case SHOOT:
                if (timer.get() > 0.5) { // TODO: Tune time
                    enterWait();
                }
                break;
            case ENDGAME:
                // TODO: Later
                break;
        }
    }
}
