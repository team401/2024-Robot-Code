package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.utils.InterpolateDouble;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ScoringSubsystem extends SubsystemBase {
    private final ShooterIO shooterIo;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final AimerIO aimerIo;
    private final AimerIOInputsAutoLogged aimerInputs = new AimerIOInputsAutoLogged();

    private final HoodIO hoodIo;
    private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

    private final Timer shootTimer = new Timer();

    private final Supplier<Pose2d> poseSupplier;

    private final InterpolateDouble shooterInterpolated;
    private final InterpolateDouble aimerInterpolated;

    private final Mechanism2d mechanism = new Mechanism2d(2.2, 1.2);
    private final MechanismRoot2d rootMechanism = mechanism.getRoot("scoring", 1.1, 0.1);
    private final MechanismLigament2d aimMechanism =
            rootMechanism.append(new MechanismLigament2d("aimer", 1.0, 0.0));
    private final MechanismLigament2d hoodMechanism =
            aimMechanism.append(
                    new MechanismLigament2d("hood", 0.1, 0.0, 10.0, new Color8Bit(0, 200, 50)));

    private enum ScoringState {
        IDLE,
        INTAKE,
        PRIME,
        AMP_PRIME,
        SHOOT,
        AMP_SHOOT,
        ENDGAME
    }

    public enum ScoringAction {
        WAIT,
        INTAKE,
        AIM,
        AMP_AIM,
        SHOOT,
        AMP_SHOOT,
        ENDGAME
    }

    private ScoringState state = ScoringState.IDLE;

    private ScoringAction action = ScoringAction.WAIT;

    public ScoringSubsystem(
            ShooterIO shooterIo, AimerIO aimerIo, HoodIO hoodIo, Supplier<Pose2d> poseSupplier) {
        this.shooterIo = shooterIo;
        this.aimerIo = aimerIo;
        this.hoodIo = hoodIo;

        this.poseSupplier = poseSupplier;

        shooterInterpolated = new InterpolateDouble(ScoringConstants.getShooterMap());

        aimerInterpolated = new InterpolateDouble(ScoringConstants.getAimerMap());
    }

    public void setAction(ScoringAction action) {
        this.action = action;
    }

    private void idle() {
        aimerIo.setAimAngleRad(0);
        shooterIo.setShooterVelocityRPM(0);
        shooterIo.setKickerVolts(0);
        hoodIo.setHoodAngleRad(0);

        Logger.recordOutput("scoring/aimGoal", 0.0);

        if (!hasNote() && action == ScoringAction.INTAKE) {
            state = ScoringState.INTAKE;
        } else if (action == ScoringAction.AIM) {
            state = ScoringState.PRIME;
            aimerIo.setAimAngleRad(Math.PI / 2);
        } else if (action == ScoringAction.AMP_AIM) {
            state = ScoringState.AMP_PRIME;
        } else if (action == ScoringAction.ENDGAME) {
            // state = ScoringState.ENDGAME; TODO: Later
        }
    }

    private void intake() {
        double closestAngleRad = aimerInputs.aimAngleRad;
        if (!canIntake()) {
            closestAngleRad =
                    aimerInputs.aimAngleRad < Math.PI / 2 - ScoringConstants.intakeAngleTolerance
                            ? Math.PI - ScoringConstants.intakeAngleTolerance
                            : Math.PI + ScoringConstants.intakeAngleTolerance;
        }
        aimerIo.setAimAngleRad(closestAngleRad);
        shooterIo.setShooterVelocityRPM(-10);
        shooterIo.setKickerVolts(-1);
        hoodIo.setHoodAngleRad(0);

        if (hasNote() || action == ScoringAction.WAIT) {
            state = ScoringState.IDLE;
        }
    }

    private void prime() {
        double distancetoGoal = findDistanceToGoal();
        Logger.recordOutput("scoring/aimGoal", aimerInterpolated.getValue(distancetoGoal));
        shooterIo.setShooterVelocityRPM(shooterInterpolated.getValue(distancetoGoal));
        aimerIo.followAimAngleRad(aimerInterpolated.getValue(distancetoGoal));

        boolean shooterReady =
                Math.abs(shooterInputs.shooterVelocityRPM - shooterInputs.shooterGoalVelocityRPM)
                        < ScoringConstants.shooterVelocityRPMMargin; // TODO: Tune
        boolean aimReady =
                Math.abs(aimerInputs.aimAngleRad - aimerInputs.aimGoalAngleRad)
                        < ScoringConstants.aimAngleRadiansMargin; // TODO: Tune
        boolean driveReady = true; // TODO: Add drive ready
        boolean notePresent = hasNote();

        boolean primeReady = shooterReady && aimReady && driveReady && notePresent;

        if (action == ScoringAction.WAIT) {
            state = ScoringState.IDLE;
        } else if (action == ScoringAction.SHOOT && primeReady) {
            state = ScoringState.SHOOT;

            shootTimer.reset();
            shootTimer.start();
        }
    }

    private void ampPrime() {
        shooterIo.setShooterVelocityRPM(ScoringConstants.shooterAmpVelocityRPM);
        aimerIo.setAimAngleRad(Math.PI / 2);
        hoodIo.setHoodAngleRad(Math.PI / 2);

        boolean shooterReady =
                Math.abs(shooterInputs.shooterVelocityRPM - shooterInputs.shooterGoalVelocityRPM)
                        < ScoringConstants.shooterVelocityRPMMargin; // TODO: Tune
        boolean aimReady =
                Math.abs(aimerInputs.aimAngleRad - aimerInputs.aimGoalAngleRad)
                        < ScoringConstants.aimAngleRadiansMargin; // TODO: Tune
        boolean hoodReady =
                Math.abs(hoodInputs.hoodAngleRad - hoodInputs.hoodGoalAngleRad)
                        < ScoringConstants.hoodAngleRadiansMargin; // TODO: Tune
        boolean driveReady = true; // TODO: Add drive ready
        boolean notePresent = hasNote();

        boolean primeReady = shooterReady && aimReady && hoodReady && driveReady && notePresent;

        if (action == ScoringAction.WAIT) {
            state = ScoringState.IDLE;
        } else if (action == ScoringAction.SHOOT && primeReady) {
            state = ScoringState.AMP_SHOOT;

            shootTimer.reset();
            shootTimer.start();
        }
    }

    private void shoot() {
        double distancetoGoal = findDistanceToGoal();
        shooterIo.setShooterVelocityRPM(shooterInterpolated.getValue(distancetoGoal));
        aimerIo.followAimAngleRad(aimerInterpolated.getValue(distancetoGoal));

        shooterIo.setKickerVolts(5);

        if (shootTimer.get() > 0.5) { // TODO: Tune time
            state = ScoringState.PRIME;

            shootTimer.stop();
        }
    }

    private void ampShoot() {
        shooterIo.setKickerVolts(5);

        if (shootTimer.get() > 0.5) { // TODO: Tune time
            state = ScoringState.AMP_PRIME;

            shootTimer.stop();
        }
    }

    private void endgame() { // TODO: Later
        state = ScoringState.IDLE;
    }

    private double findDistanceToGoal() {
        Translation2d speakerPose = FieldConstants.speakerPose;
        Pose2d robotPose = poseSupplier.get();
        double distancetoGoal =
                Math.sqrt(
                        Math.pow(Math.abs(robotPose.getX() - speakerPose.getX()), 2)
                                + Math.pow(Math.abs(robotPose.getY() - speakerPose.getY()), 2));
        Logger.recordOutput("scoring/distance", distancetoGoal);
        return distancetoGoal;
    }

    public boolean hasNote() {
        return shooterInputs.bannerSensor;
    }

    public boolean canIntake() {
        return Math.abs(aimerInputs.aimAngleRad - Math.PI / 2)
                < ScoringConstants.intakeAngleTolerance;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("scoring/State", state.toString());
        Logger.recordOutput("scoring/Action", action.toString());

        shooterIo.updateInputs(shooterInputs);
        aimerIo.updateInputs(aimerInputs);
        hoodIo.updateInputs(hoodInputs);

        Logger.processInputs("scoring/shooter", shooterInputs);
        Logger.processInputs("scoring/aimer", aimerInputs);
        Logger.processInputs("scoring/hood", hoodInputs);

        aimMechanism.setAngle(Units.radiansToDegrees(aimerInputs.aimAngleRad));
        hoodMechanism.setAngle(Units.radiansToDegrees(hoodInputs.hoodAngleRad));
        Logger.recordOutput("scoring/mechanism2d", mechanism);

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
            case AMP_PRIME:
                ampPrime();
                break;
            case SHOOT:
                shoot();
                break;
            case AMP_SHOOT:
                ampShoot();
                break;
            case ENDGAME:
                endgame(); // TODO: Later
                break;
        }
    }
}
