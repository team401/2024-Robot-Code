package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public enum ScoringAction {
        INTAKE,
        AIM,
        SHOOT,
        ABORT,
        ENDGAME
    }

    private ScoringState state = ScoringState.WAIT;

    public ScoringSubsystem(ShooterIO shooterIo, AimerIO aimerIo) {
        this.shooterIo = shooterIo;
        this.aimerIo = aimerIo;
    }

    public void setAction(ScoringAction action) {
        switch (action) {
            case INTAKE:
                if (state == ScoringState.WAIT) {
                    state = ScoringState.INTAKE;

                    aimerIo.setAimAngRad(0);
                    shooterIo.setShooterVelRPM(-10);
                    shooterIo.setKickerVolts(-1);
                }
                break;
            case AIM:
                if (state == ScoringState.WAIT && true) { // TODO: Add banner sensor
                    state = ScoringState.PRIME;

                    shooterIo.setShooterVelRPM(100);
                }
                break;
            case SHOOT:
                if (state == ScoringState.PRIME) {
                    state = ScoringState.SHOOT;

                    shooterIo.setKickerVolts(1);
                    timer.reset();
                    timer.start();
                }
                break;
            case ABORT:
                state = ScoringState.WAIT;

                aimerIo.setAimAngRad(0);
                shooterIo.setShooterVelRPM(0);
                shooterIo.setKickerVolts(0);
                break;
            case ENDGAME:
                if (state == ScoringState.WAIT) {
                    state = ScoringState.ENDGAME;
                }
                break;
        }
    }

    private void abort() {}

    private void intake() {
        if (true) { // TODO: Banner sensor triggered
            setAction(ScoringAction.ABORT);
        }
    }

    private void aim() {
        aimerIo.setAimAngRad(findShootAngleRads());
    }

    private void shoot() {
        if (timer.get() > 0.5) { // TODO: Tune time
            setAction(ScoringAction.ABORT);
        }
    }

    private void endgame() {} // TODO: Later

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
                abort();
                break;
            case INTAKE:
                intake();
                break;
            case PRIME:
                aim();
                break;
            case SHOOT:
                shoot();
                break;
            case ENDGAME:
                endgame();// TODO: Later
                break;
        }
    }
}
