package frc.robot.subsystems.scoring;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringConstants;
import frc.robot.utils.FieldFinder;
import frc.robot.utils.FieldFinder.FieldLocations;
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

    private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
    private Supplier<Vector<N2>> velocitySupplier = () -> VecBuilder.fill(0.0, 0.0);
    private Supplier<Translation2d> speakerSupplier = () -> new Translation2d(0, 0);
    private Supplier<Double> elevatorPositionSupplier = () -> 0.0;
    private Supplier<Boolean> driveAllignedSupplier = () -> true;

    private final InterpolateDouble shooterInterpolated;
    private final InterpolateDouble aimerInterpolated;
    private final InterpolateDouble timeToPutAimDown;

    private double shooterGoalVelocityRPMTuning = 0.0;
    private double aimerGoalAngleRadTuning = 0.0;
    private double kickerVoltsTuning = 0.0;

    private final Mechanism2d mechanism = new Mechanism2d(2.2, 2.0);
    private final MechanismRoot2d rootMechanism = mechanism.getRoot("scoring", 0.6, 0.3);
    private final MechanismLigament2d aimMechanism =
            rootMechanism.append(new MechanismLigament2d("aimer", 0.5, 0.0));
    private final MechanismLigament2d hoodMechanism =
            aimMechanism.append(
                    new MechanismLigament2d("hood", 0.2, 0.0, 10.0, new Color8Bit(0, 200, 50)));

    public enum ScoringState {
        IDLE,
        INTAKE,
        PRIME,
        AMP_PRIME,
        SHOOT,
        AMP_SHOOT,
        ENDGAME,
        TUNING
    }

    public enum ScoringAction {
        WAIT,
        INTAKE,
        AIM,
        AMP_AIM,
        SHOOT,
        ENDGAME,
        TUNING
    }

    private ScoringState state = ScoringState.IDLE;

    private ScoringAction action = ScoringAction.WAIT;

    private boolean readyToShoot = false;

    public ScoringSubsystem(ShooterIO shooterIo, AimerIO aimerIo, HoodIO hoodIo) {

        this.shooterIo = shooterIo;
        this.aimerIo = aimerIo;
        this.hoodIo = hoodIo;

        shooterInterpolated = new InterpolateDouble(ScoringConstants.getShooterMap());

        aimerInterpolated =
                new InterpolateDouble(
                        ScoringConstants.getAimerMap(), 0.0, ScoringConstants.aimMaxAngleRadians);

        timeToPutAimDown = new InterpolateDouble(ScoringConstants.timeToPutAimDownMap(), 0.0, 2.0);
    }

    public void setAction(ScoringAction action) {
        this.action = action;
    }

    private void idle() {
        aimerIo.setAimAngleRad(0, true);
        shooterIo.setShooterVelocityRPM(0);
        shooterIo.setKickerVolts(0);
        hoodIo.setHoodAngleRad(0);

        Logger.recordOutput("scoring/aimGoal", 0.0);

        if (!hasNote() && action == ScoringAction.INTAKE) {
            state = ScoringState.INTAKE;
        } else if (action == ScoringAction.AIM || action == ScoringAction.SHOOT) {
            state = ScoringState.PRIME;
            aimerIo.setAimAngleRad(Math.PI / 4, true);
        } else if (action == ScoringAction.AMP_AIM) {
            state = ScoringState.AMP_PRIME;
        } else if (action == ScoringAction.ENDGAME) {
            // state = ScoringState.ENDGAME; TODO: Later
        } else if (action == ScoringAction.TUNING) {
            state = ScoringState.TUNING;
            SmartDashboard.putNumber("Tuning/AimerGoal", aimerGoalAngleRadTuning);
            SmartDashboard.putNumber("Tuning/ShooterGoal", shooterGoalVelocityRPMTuning);
        }
    }

    private void intake() {
        if (!canIntake()) {
            aimerIo.setAimAngleRad(ScoringConstants.intakeAngleToleranceRadians, true);
        }
        shooterIo.setKickerVolts(0);
        hoodIo.setHoodAngleRad(0);

        if (hasNote() || action == ScoringAction.WAIT) {
            state = ScoringState.IDLE;
        }
    }

    private void prime() {
        double distancetoGoal = findDistanceToGoal();
        Logger.recordOutput("scoring/aimGoal", aimerInterpolated.getValue(distancetoGoal));
        shooterIo.setShooterVelocityRPM(shooterInterpolated.getValue(distancetoGoal));
        aimerIo.setAimAngleRad(aimerInterpolated.getValue(distancetoGoal), false);

        boolean shooterReady =
                Math.abs(
                                shooterInputs.shooterLeftVelocityRPM
                                        - shooterInputs.shooterLeftGoalVelocityRPM)
                        < ScoringConstants.shooterVelocityMarginRPM; // TODO: Tune
        boolean aimReady =
                Math.abs(aimerInputs.aimAngleRad - aimerInputs.aimGoalAngleRad)
                        < ScoringConstants.aimAngleMarginRadians; // TODO: Tune
        boolean driveReady = true; // TODO: Add drive ready
        boolean notePresent = hasNote();

        boolean primeReady = shooterReady && aimReady && driveReady && notePresent;
        readyToShoot = primeReady;

        if (action != ScoringAction.SHOOT && action != ScoringAction.AIM) {
            state = ScoringState.IDLE;
        } else if (action == ScoringAction.SHOOT && primeReady) {
            state = ScoringState.SHOOT;

            shootTimer.reset();
            shootTimer.start();
            readyToShoot = false;
        }
    }

    private void ampPrime() {
        shooterIo.setShooterVelocityRPM(ScoringConstants.shooterAmpVelocityRPM);
        aimerIo.setAimAngleRad(Math.PI / 2, true);
        hoodIo.setHoodAngleRad(Math.PI);

        boolean shooterReady =
                Math.abs(
                                shooterInputs.shooterLeftVelocityRPM
                                        - shooterInputs.shooterLeftGoalVelocityRPM)
                        < ScoringConstants.shooterVelocityMarginRPM; // TODO: Tune
        boolean aimReady =
                Math.abs(aimerInputs.aimAngleRad - aimerInputs.aimGoalAngleRad)
                        < ScoringConstants.aimAngleMarginRadians; // TODO: Tune
        boolean hoodReady =
                Math.abs(hoodInputs.hoodAngleRad - hoodInputs.hoodGoalAngleRad)
                        < ScoringConstants.hoodAngleMarginRadians; // TODO: Tune
        boolean driveReady = driveAllignedSupplier.get();
        boolean notePresent = hasNote();

        boolean primeReady = shooterReady && aimReady && hoodReady && driveReady && notePresent;
        readyToShoot = primeReady;

        if (action != ScoringAction.SHOOT && action != ScoringAction.AMP_AIM) {
            state = ScoringState.IDLE;
        } else if (action == ScoringAction.SHOOT && primeReady) {
            state = ScoringState.AMP_SHOOT;

            shootTimer.reset();
            shootTimer.start();
            readyToShoot = false;
        }
    }

    private void shoot() {
        double distancetoGoal = findDistanceToGoal();
        shooterIo.setShooterVelocityRPM(shooterInterpolated.getValue(distancetoGoal));
        aimerIo.setAimAngleRad(aimerInterpolated.getValue(distancetoGoal), false);

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

    private void tuning() {
        shooterGoalVelocityRPMTuning = SmartDashboard.getNumber("Tuning/ShooterGoal", 0.0);
        aimerGoalAngleRadTuning = SmartDashboard.getNumber("Tuning/AimerGoal", 0.0);
        shooterIo.setShooterVelocityRPM(shooterGoalVelocityRPMTuning);
        aimerIo.setAimAngleRad(aimerGoalAngleRadTuning, false);
        hoodIo.setHoodAngleRad(0.0);
        shooterIo.setKickerVolts(kickerVoltsTuning);

        if (action != ScoringAction.TUNING) {
            state = ScoringState.IDLE;
        }
    }

    private double findDistanceToGoal() {
        Translation2d speakerPose = speakerSupplier.get();
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
        return aimerInputs.aimAngleRad > ScoringConstants.intakeAngleToleranceRadians;
    }

    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    public void setVelocitySupplier(Supplier<Vector<N2>> velocitySupplier) {
        this.velocitySupplier = velocitySupplier;
    }

    public void setSpeakerSupplier(Supplier<Translation2d> speakerSupplier) {
        this.speakerSupplier = speakerSupplier;
    }

    public void setElevatorPositionSupplier(Supplier<Double> elevatorPositionSupplier) {
        this.elevatorPositionSupplier = elevatorPositionSupplier;
    }

    public void setDriveAllignedSupplier(Supplier<Boolean> driveAllignedSupplier) {
        this.driveAllignedSupplier = driveAllignedSupplier;
    }

    @Override
    public void periodic() {
        if (state != ScoringState.TUNING
                && action != ScoringAction.ENDGAME
                && (FieldFinder.willIHitThis(
                                poseSupplier.get().getX(),
                                poseSupplier.get().getY(),
                                velocitySupplier.get().get(0, 0)
                                        * timeToPutAimDown.getValue(aimerInputs.aimAngleRad),
                                velocitySupplier.get().get(1, 0)
                                        * timeToPutAimDown.getValue(aimerInputs.aimAngleRad),
                                FieldLocations.BLUE_STAGE)
                        || FieldFinder.willIHitThis(
                                poseSupplier.get().getX(),
                                poseSupplier.get().getY(),
                                velocitySupplier.get().get(0, 0)
                                        * timeToPutAimDown.getValue(aimerInputs.aimAngleRad),
                                velocitySupplier.get().get(1, 0)
                                        * timeToPutAimDown.getValue(aimerInputs.aimAngleRad),
                                FieldLocations.RED_STAGE))) {
            aimerIo.setAngleClampsRad(0, 0);
        } else {
            // Linear equation in point-slope form to calculate the arm's limit based on the
            // elevator position
            double maxElevatorPosition = ScoringConstants.maxElevatorPosition; // x-intercept
            double maxAimAngle = ScoringConstants.maxAimAngleElevatorLimit; // y-intercept

            double elevatorLimit =
                    (maxAimAngle / maxElevatorPosition)
                                    * (elevatorPositionSupplier.get() - maxElevatorPosition)
                            + maxAimAngle;
            aimerIo.setAngleClampsRad(
                    Math.max(0.0, elevatorLimit), ScoringConstants.aimMaxAngleRadians);
        }

        aimerIo.controlAimAngleRad();

        Logger.recordOutput("scoring/State", state.toString());
        Logger.recordOutput("scoring/Action", action.toString());

        shooterIo.updateInputs(shooterInputs);
        aimerIo.updateInputs(aimerInputs);
        hoodIo.updateInputs(hoodInputs);

        Logger.processInputs("scoring/shooter", shooterInputs);
        Logger.processInputs("scoring/aimer", aimerInputs);
        Logger.processInputs("scoring/hood", hoodInputs);

        Logger.recordOutput(
                "scoring/Aimer3d",
                new Pose3d(-0.255, 0.2, 0.502, new Rotation3d(0, -aimerInputs.aimAngleRad, 0)));
        Logger.recordOutput(
                "scoring/Hood3d",
                new Pose3d(
                        0.501360149992 * Math.cos(aimerInputs.aimAngleRad) - 0.255,
                        // Independent position: 0.246 (leave for now, might be used later)
                        0.193,
                        0.501360149992 * Math.sin(aimerInputs.aimAngleRad) + 0.483,
                        // Independent position: 0.483 (leave for now, might be used later)
                        new Rotation3d(0, -hoodInputs.hoodAngleRad - aimerInputs.aimAngleRad, 0)));

        aimMechanism.setAngle(Units.radiansToDegrees(aimerInputs.aimAngleRad));
        hoodMechanism.setAngle(Units.radiansToDegrees(hoodInputs.hoodAngleRad));
        Logger.recordOutput("scoring/mechanism2d", mechanism);

        switch (state) {
            case IDLE:
                idle();
                SmartDashboard.putString("shoot", "idle");
                break;
            case INTAKE:
                intake();
                SmartDashboard.putString("shoot", "intake");
                break;
            case PRIME:
                prime();
                SmartDashboard.putString("shoot", "prime");
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
            case TUNING:
                tuning();
                break;
        }
    }

    public void setTuningKickerVolts(double kickerVoltsTuning) {
        this.kickerVoltsTuning = kickerVoltsTuning;
    }

    public ScoringAction getCurrentAction() {
        return action;
    }

    public ScoringState getCurrentState() {
        return state;
    }

    public boolean readyToShoot() {
        return readyToShoot;
    }
}
