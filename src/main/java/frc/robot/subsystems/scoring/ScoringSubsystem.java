package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringSubsystem extends SubsystemBase {
    private final ShooterIO shooterIo;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final AimerIO aimerIo;
    private final AimerIOInputsAutoLogged aimerInputs = new AimerIOInputsAutoLogged();

    private final Timer shootTimer = new Timer();

    private enum ScoringState {
        IDLE,
        INTAKE,
        PRIME,
        SHOOT,
        ENDGAME
    }

    public enum ScoringAction {
        INTAKE,
        AIM,
        SHOOT,
        WAIT,
        ENDGAME
    }

    private ScoringState state = ScoringState.IDLE;

    private ScoringAction action = ScoringAction.WAIT;

    public ScoringSubsystem(ShooterIO shooterIo, AimerIO aimerIo) {
        this.shooterIo = shooterIo;
        this.aimerIo = aimerIo;
    }

    public void setAction(ScoringAction action) {
        this.action = action;
    }

    private void idle() {
        aimerIo.setAimAngleRad(0);
        shooterIo.setShooterVelocityRPM(0);
        shooterIo.setKickerVolts(0);

        if (true && action == ScoringAction.INTAKE) { // TODO: Banner sensor NOT triggered
            state = ScoringState.INTAKE;
        } else if (action == ScoringAction.AIM) {
            state = ScoringState.PRIME;
        } else if (action == ScoringAction.ENDGAME) {
            state = ScoringState.ENDGAME;
        }
    }

    private void intake() {
        aimerIo.setAimAngleRad(0);
        shooterIo.setShooterVelocityRPM(-10);
        shooterIo.setKickerVolts(-1);

        if (true || action == ScoringAction.WAIT) { // TODO: Banner sensor triggered
            state = ScoringState.IDLE;
        }
    }

    private void prime() {
        shooterIo.setShooterVelocityRPM(100);
        aimerIo.setAimAngleRad(findShootAngleRads());

        boolean shooterReady = Math.abs(shooterInputs.shooterVelocityRPM - shooterInputs.shooterGoalVelocityRPM) < 10; // TODO: Tune
        boolean armReady = Math.abs(aimerInputs.aimAngleRad - aimerInputs.aimGoalAngleRad) < 0.1; // TODO: Tune
        boolean driveReady = true; // TODO: Add drive ready
        boolean hasNote = true; // TODO: Add banner sensor

        boolean primeReady = shooterReady && armReady && driveReady && hasNote;

        if (action == ScoringAction.WAIT) {
            state = ScoringState.IDLE;
        } else if (action == ScoringAction.SHOOT && primeReady) {
            state = ScoringState.SHOOT;

            shootTimer.reset();
            shootTimer.start();
        }
    }

    private void shoot() {
        shooterIo.setKickerVolts(5);

        if (shootTimer.get() > 0.5) { // TODO: Tune time
            state = ScoringState.PRIME;

            shootTimer.stop();
        }
    }

    private void endgame() { // TODO: Later
        state = ScoringState.IDLE;
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
            case IDLE:
                idle();
                break;
            case INTAKE:
                intake();
                break;
            case PRIME:
                prime();
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
