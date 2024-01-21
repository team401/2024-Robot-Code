package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.utils.InterpolateDouble;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ScoringSubsystem extends SubsystemBase {
    private final ShooterIO shooterIo;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final AimerIO aimerIo;
    private final AimerIOInputsAutoLogged aimerInputs = new AimerIOInputsAutoLogged();

    DigitalInput bannerSensor = new DigitalInput(Constants.SensorConstants.bannerPort);

    private final Timer shootTimer = new Timer();

    private final Supplier<Pose2d> poseSupplier;

    private final InterpolateDouble shooterInterpolated;
    private final InterpolateDouble aimerInterpolated;

    private enum ScoringState {
        IDLE,
        INTAKE,
        PRIME,
        AMP_PRIME,
        SHOOT,
        ENDGAME,
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

    public ScoringSubsystem(ShooterIO shooterIo, AimerIO aimerIo, Supplier<Pose2d> poseSupplier) {
        this.shooterIo = shooterIo;
        this.aimerIo = aimerIo;

        this.poseSupplier = poseSupplier;

        shooterInterpolated = new InterpolateDouble(ScoringConstants.getAimerMap());

        aimerInterpolated = new InterpolateDouble(ScoringConstants.getShooterMap());
    }

    public void setAction(ScoringAction action) {
        this.action = action;
    }

    private void idle() {
        aimerIo.setAimAngleRad(0);
        shooterIo.setShooterVelocityRPM(0);
        shooterIo.setKickerVolts(0);

        if (!hasNote() && action == ScoringAction.INTAKE) {
            state = ScoringState.INTAKE;
        } else if (action == ScoringAction.AIM) {
            state = ScoringState.PRIME;
        } else if (action == ScoringAction.ENDGAME) {
            // state = ScoringState.ENDGAME; TODO: Later
        }
    }

    private void intake() {
        aimerIo.setAimAngleRad(0);
        shooterIo.setShooterVelocityRPM(-10);
        shooterIo.setKickerVolts(-1);

        if (hasNote() || action == ScoringAction.WAIT) {
            state = ScoringState.IDLE;
        }
    }

    private void prime() {
        double distancetoGoal = findDistanceToGoal();
        shooterIo.setShooterVelocityRPM(shooterInterpolated.getValue(distancetoGoal));
        aimerIo.setAimAngleRad(aimerInterpolated.getValue(distancetoGoal));

        boolean shooterReady =
                Math.abs(shooterInputs.shooterVelocityRPM - shooterInputs.shooterGoalVelocityRPM)
                        < 10; // TODO: Tune
        boolean aimReady =
                Math.abs(aimerInputs.aimAngleRad - aimerInputs.aimGoalAngleRad)
                        < 0.01; // TODO: Tune
        boolean driveReady = true; // TODO: Add drive ready
        boolean hasNote = hasNote();

        boolean primeReady = shooterReady && aimReady && driveReady && hasNote;

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

    private double findDistanceToGoal() {
        Translation2d speakerPose =
                false // TODO: CHANGE THIS URGENT
                        // DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                        ? new Translation2d(
                                Units.inchesToMeters(652.73), Units.inchesToMeters(218.42))
                        : new Translation2d(
                                Units.inchesToMeters(-1.5),
                                Units.inchesToMeters(218.42)); // TODO: Might have to change these
        Pose2d robotPose = poseSupplier.get();
        double distancetoGoal =
                Math.sqrt(
                        Math.pow(Math.abs(robotPose.getX() - speakerPose.getX()), 2)
                                + Math.pow(
                                        Math.abs(robotPose.getY() - speakerPose.getY()),
                                        2)); // TODO: Change to be more accurate
        Logger.recordOutput("scoring/distance", distancetoGoal);
        return distancetoGoal;
    }

    public boolean hasNote() {
        return bannerSensor.get();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("scoring/State", state.toString());
        Logger.recordOutput("scoring/Action", action.toString());

        shooterIo.updateInputs(shooterInputs);
        aimerIo.updateInputs(aimerInputs);

        Logger.processInputs("scoring/shooter", shooterInputs);
        Logger.processInputs("scoring/aimer", aimerInputs);

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
                endgame(); // TODO: Later
                break;
        }
    }
}
