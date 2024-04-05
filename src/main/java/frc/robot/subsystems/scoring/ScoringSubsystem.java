package frc.robot.subsystems.scoring;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ScoringConstants;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.FieldFinder;
import frc.robot.utils.FieldFinder.FieldLocations;
import frc.robot.utils.InterpolateDouble;
import frc.robot.utils.Tunable;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ScoringSubsystem extends SubsystemBase implements Tunable {
    private final ShooterIO shooterIo;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final AimerIO aimerIo;
    private final AimerIOInputsAutoLogged aimerInputs = new AimerIOInputsAutoLogged();

    private final HoodIO hoodIo;
    private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

    private final Timer shootTimer = new Timer();

    private final Timer sourceIntakeTimer = new Timer();
    private boolean sourceTimerStarted = false;

    private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
    private Supplier<Vector<N2>> velocitySupplier = () -> VecBuilder.fill(0.0, 0.0);
    private DoubleSupplier elevatorPositionSupplier = () -> 0.0;
    private Supplier<Boolean> driveAllignedSupplier = () -> true;

    private final InterpolateDouble shooterInterpolated;
    private final InterpolateDouble aimerInterpolated;
    private final InterpolateDouble timeToPutAimDown;
    private final InterpolateDouble aimerAvoidElevator;
    private final InterpolateDouble aimerAngleTolerance;

    private double shooterGoalVelocityRPMTuning = 0.0;
    private double aimerGoalAngleRadTuning = 0.0;
    private double kickerVoltsTuning = 0.0;

    private boolean overrideIntake = false;
    private boolean overrideShoot = false;
    private boolean overrideStageAvoidance = false;
    private boolean overrideBeamBreak = false;

    private boolean hoodForced = false;

    private Mechanism2d mechanism;
    private MechanismRoot2d rootMechanism;
    private MechanismLigament2d aimMechanism;
    private MechanismLigament2d hoodMechanism;

    public enum ScoringState {
        IDLE,
        INTAKE,
        SPIT,
        SOURCE_INTAKE,
        PRIME,
        AMP_PRIME,
        SHOOT,
        AMP_SHOOT,
        ENDGAME,
        TUNING,
        OVERRIDE,
        TEMPORARY_SETPOINT
    }

    public enum ScoringAction {
        WAIT,
        INTAKE,
        SPIT,
        SOURCE_INTAKE,
        AIM,
        AMP_AIM,
        SHOOT,
        ENDGAME,
        TUNING,
        OVERRIDE,
        TEMPORARY_SETPOINT,
        TRAP_SCORE
    }

    private ScoringState state = ScoringState.IDLE;

    private ScoringAction action = ScoringAction.WAIT;

    private int temporarySetpointSlot = 0;
    private double temporarySetpointPosition = 0.0;

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

        aimerAvoidElevator =
                new InterpolateDouble(
                        ScoringConstants.aimerAvoidElevatorTable(), 0.0, Math.PI / 2.0);

        aimerAngleTolerance = new InterpolateDouble(ScoringConstants.aimerToleranceTable());

        if (Constants.currentMode == Mode.SIM) {
            mechanism = new Mechanism2d(2.2, 2.0);
            rootMechanism = mechanism.getRoot("scoring", 0.6, 0.3);
            aimMechanism = rootMechanism.append(new MechanismLigament2d("aimer", 0.5, 0.0));
            hoodMechanism =
                    aimMechanism.append(
                            new MechanismLigament2d(
                                    "hood", 0.2, 0.0, 10.0, new Color8Bit(0, 200, 50)));
        }
    }

    public void setAction(ScoringAction action) {
        this.action = action;
    }

    public void setBrakeMode(boolean brake) {
        aimerIo.setBrakeMode(brake);
        hoodIo.setBrakeMode(brake);
    }

    public boolean atAimerGoalPosition() {
        return Math.abs(aimerInputs.aimAngleRad - aimerInputs.aimGoalAngleRad) < 0.2;
    }

    private void idle() {
        aimerIo.setAimAngleRad(-0.03, true);
        shooterIo.setShooterVelocityRPM(0);
        shooterIo.setKickerVolts(0);
        // hoodIo.setHoodAngleRad(0);
        hoodIo.setOverrideVolts(hoodForced ? 3 : 0);

        aimerIo.setOverrideMode(false);
        hoodIo.setOverrideMode(true); // TODO: Change
        shooterIo.setOverrideMode(false);
        shooterIo.setOverrideVolts(0);

        Logger.recordOutput("scoring/aimGoal", 0.0);

        SmartDashboard.putBoolean("Has Note", shooterInputs.bannerSensor);
        SmartDashboard.putNumber("Aimer Location", aimerInputs.aimAngleRad);

        if ((!hasNote() || overrideIntake) && action == ScoringAction.INTAKE) {
            state = ScoringState.INTAKE;
        } else if ((!hasNote() || overrideIntake) && action == ScoringAction.SOURCE_INTAKE) {
            state = ScoringState.SOURCE_INTAKE;
            sourceTimerStarted = false;
        } else if (action == ScoringAction.SPIT) {
            state = ScoringState.SPIT;
        } else if (action == ScoringAction.AIM || action == ScoringAction.SHOOT) {
            state = ScoringState.PRIME;
            aimerIo.setAimAngleRad(aimerInputs.aimAngleRad + 0.001, true);
            shooterIo.setShooterVelocityRPM(2000);
        } else if (action == ScoringAction.AMP_AIM) {
            state = ScoringState.AMP_PRIME;
        } else if (action == ScoringAction.ENDGAME || action == ScoringAction.TRAP_SCORE) {
            state = ScoringState.ENDGAME;
        } else if (action == ScoringAction.TUNING) {
            state = ScoringState.TUNING;
            SmartDashboard.putNumber("Test-Mode/AimerGoal", aimerGoalAngleRadTuning);
            SmartDashboard.putNumber("Test-Mode/ShooterGoal", shooterGoalVelocityRPMTuning);
        } else if (action == ScoringAction.OVERRIDE) {
            state = ScoringState.OVERRIDE;
        }
    }

    private void intake() {
        if (!aimerAtIntakePosition()) {
            aimerIo.setAimAngleRad(ScoringConstants.intakeAngleToleranceRadians, true);
        }
        shooterIo.setKickerVolts(ScoringConstants.kickerIntakeVolts);

        if ((hasNote()) || action != ScoringAction.INTAKE) {
            state = ScoringState.IDLE;
        }
    }

    private void sourceIntake() {
        aimerIo.setAimAngleRad(0.35, true);
        shooterIo.setKickerVolts(-1);

        shooterIo.setOverrideMode(true);
        shooterIo.setOverrideVolts(-2);

        if (hasNote() && !sourceTimerStarted) {
            sourceIntakeTimer.reset();
            sourceIntakeTimer.start();

            sourceTimerStarted = true;
        }

        if ((sourceIntakeTimer.get() > 0.15 && sourceTimerStarted)
                || action != ScoringAction.SOURCE_INTAKE) {
            sourceIntakeTimer.stop();

            shooterIo.setOverrideMode(false);
            sourceTimerStarted = false;
            state = ScoringState.IDLE;
        }
    }

    private void spit() {
        shooterIo.setKickerVolts(-4.0);

        if (action != ScoringAction.SPIT) {
            state = ScoringState.IDLE;
        }
    }

    private void prime() {
        double distanceToGoal = findDistanceToGoal();
        Logger.recordOutput("scoring/aimGoal", getAimerAngle(distanceToGoal));
        shooterIo.setShooterVelocityRPM(shooterInterpolated.getValue(distanceToGoal));
        aimerIo.setAimAngleRad(getAimerAngle(distanceToGoal), false);
        if (!overrideBeamBreak) {
            shooterIo.setKickerVolts(hasNote() ? 0.0 : ScoringConstants.kickerIntakeVolts);
        }

        boolean shooterReady =
                shooterInputs.shooterLeftVelocityRPM
                                < (shooterInputs.shooterLeftGoalVelocityRPM
                                        + ScoringConstants.shooterUpperVelocityMarginRPM)
                        && shooterInputs.shooterLeftVelocityRPM
                                > (shooterInputs.shooterLeftGoalVelocityRPM
                                        - ScoringConstants.shooterLowerVelocityMarginRPM);
        boolean aimReady =
                Math.abs(aimerInputs.aimAngleRad - aimerInputs.aimGoalAngleRad)
                                < aimerAngleTolerance.getValue(distanceToGoal)
                        && Math.abs(aimerInputs.aimVelocityErrorRadPerSec)
                                < ScoringConstants.aimAngleVelocityMargin;
        boolean driveReady = driveAllignedSupplier.get();
        boolean fieldLocationReady = true;
        if (!DriverStation.getAlliance().isPresent()) {
            fieldLocationReady = true;
        } else {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    fieldLocationReady =
                            FieldFinder.whereAmI(poseSupplier.get()) == FieldLocations.BLUE_WING
                                    || (FieldFinder.whereAmI(poseSupplier.get())
                                                    == FieldLocations.MIDDLE
                                            && DriverStation.isTeleop());
                    break;
                case Red:
                    fieldLocationReady =
                            FieldFinder.whereAmI(poseSupplier.get()) == FieldLocations.RED_WING
                                    || (FieldFinder.whereAmI(poseSupplier.get())
                                                    == FieldLocations.MIDDLE
                                            && DriverStation.isTeleop());
                    break;
            }
        }
        boolean notePresent = overrideBeamBreak ? true : hasNote();

        boolean primeReady = shooterReady && aimReady && driveReady && fieldLocationReady;
        readyToShoot = primeReady && notePresent;

        Logger.recordOutput("scoring/shooterReady", shooterReady);
        Logger.recordOutput("scoring/aimReady", aimReady);
        Logger.recordOutput("scoring/driverReady", driveReady);
        Logger.recordOutput("scoring/fieldLocationReady", fieldLocationReady);
        Logger.recordOutput("scoring/notePresent", notePresent);
        Logger.recordOutput("scoring/primeReady", primeReady);
        Logger.recordOutput("scoring/readyToShoot", readyToShoot);

        if (action != ScoringAction.SHOOT && action != ScoringAction.AIM) {
            state = ScoringState.IDLE;
        } else if (action == ScoringAction.SHOOT && (readyToShoot || overrideShoot)) {
            state = ScoringState.SHOOT;

            shootTimer.reset();
            shootTimer.start();
        }
    }

    private void ampPrime() {
        shooterIo.setShooterVelocityRPM(ScoringConstants.shooterAmpVelocityRPM);
        aimerIo.setAimAngleRad(1.65, true);
        // hoodIo.setHoodAngleRad(Math.PI);
        hoodIo.setOverrideVolts(3);

        if (action != ScoringAction.SHOOT && action != ScoringAction.AMP_AIM) {
            state = ScoringState.IDLE;
        } else if (action == ScoringAction.SHOOT) {
            state = ScoringState.AMP_SHOOT;

            shootTimer.reset();
            shootTimer.start();
        }
    }

    private void shoot() {
        double distancetoGoal = findDistanceToGoal();
        shooterIo.setShooterVelocityRPM(shooterInterpolated.getValue(distancetoGoal));
        aimerIo.setAimAngleRad(getAimerAngle(distancetoGoal), false);

        shooterIo.setKickerVolts(10);

        if (shootTimer.get() > 0.5) { // TODO: Tune time
            state = ScoringState.PRIME;

            shootTimer.stop();
        }
    }

    private void ampShoot() {
        shooterIo.setKickerVolts(10);

        if (shootTimer.get() > 1.0) { // TODO: Tune time
            state = ScoringState.AMP_PRIME;

            shootTimer.stop();
        }
    }

    private void endgame() {
        aimerIo.setAimAngleRad(Math.PI / 2, true);
        shooterIo.setShooterVelocityRPM(ScoringConstants.shooterAmpVelocityRPM);
        shooterIo.setKickerVolts(action == ScoringAction.TRAP_SCORE ? 10 : 0);
        // hoodIo.setHoodAngleRad(0);
        hoodIo.setOverrideVolts(3);
        if (action != ScoringAction.ENDGAME && action != ScoringAction.TRAP_SCORE) {
            state = ScoringState.IDLE;
        }
    }

    private void tuning() {
        shooterGoalVelocityRPMTuning = SmartDashboard.getNumber("Test-Mode/ShooterGoal", 0.0);
        aimerGoalAngleRadTuning = SmartDashboard.getNumber("Test-Mode/AimerGoal", 0.0);
        shooterIo.setShooterVelocityRPM(shooterGoalVelocityRPMTuning);
        aimerIo.setAimAngleRad(aimerGoalAngleRadTuning, true);
        // hoodIo.setHoodAngleRad(0.0);
        hoodIo.setOverrideVolts(0);
        shooterIo.setKickerVolts(kickerVoltsTuning);

        if (action != ScoringAction.TUNING) {
            state = ScoringState.IDLE;
        }
    }

    private void override() {
        shooterIo.setKickerVolts(kickerVoltsTuning);

        aimerIo.setOverrideMode(true);
        hoodIo.setOverrideMode(true); // TODO: Change
        shooterIo.setOverrideMode(true);

        if (action == ScoringAction.TEMPORARY_SETPOINT) {
            state = ScoringState.TEMPORARY_SETPOINT;
        }

        if (action != ScoringAction.OVERRIDE) {
            state = ScoringState.IDLE;
        }
    }

    private void temporarySetpoint() {
        shooterIo.setKickerVolts(kickerVoltsTuning);

        aimerIo.setOverrideMode(false);
        hoodIo.setOverrideMode(true); // TODO: Change
        shooterIo.setOverrideMode(false);

        // if (MathUtil.isNear(temporarySetpointPosition, getPosition(temporarySetpointSlot), 0.1)
        //         && MathUtil.isNear(0.0, getVelocity(temporarySetpointSlot), 0.01)) {
        if (action != ScoringAction.TEMPORARY_SETPOINT) {
            state = ScoringState.OVERRIDE;

            setVolts(0.0, temporarySetpointSlot);
        }
    }

    private double findDistanceToGoal() {
        Translation2d speakerPose = AllianceUtil.getFieldToSpeaker();
        Pose2d robotPose = poseSupplier.get();
        double distancetoGoal =
                Math.sqrt(
                        Math.pow(Math.abs(robotPose.getX() - speakerPose.getX()), 2)
                                + Math.pow(Math.abs(robotPose.getY() - speakerPose.getY()), 2));
        return distancetoGoal;
    }

    public boolean hasNote() {
        return shooterInputs.bannerSensor;
    }

    public boolean aimerAtIntakePosition() {
        return aimerInputs.aimAngleRad > ScoringConstants.intakeAngleToleranceRadians;
        // return true;\][]
    }

    public boolean canIntake() {
        return aimerAtIntakePosition() && !hasNote();
    }

    public void homeHood() {
        hoodIo.home();
    }

    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    public void setVelocitySupplier(Supplier<Vector<N2>> velocitySupplier) {
        this.velocitySupplier = velocitySupplier;
    }

    public void setElevatorPositionSupplier(DoubleSupplier elevatorPositionSupplier) {
        this.elevatorPositionSupplier = elevatorPositionSupplier;
    }

    public void setDriveAllignedSupplier(Supplier<Boolean> driveAllignedSupplier) {
        this.driveAllignedSupplier = driveAllignedSupplier;
    }

    private boolean willHitStage() {
        return (FieldFinder.willIHitThis(
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
                        FieldLocations.RED_STAGE));
    }

    public void enabledInit() {
        aimerIo.resetPID();

        setOverrideStageAvoidance(false);
        setOverrideShoot(false);
        aimerIo.setAimAngleRad(aimerInputs.aimAngleRad, true);
    }

    @Override
    public void periodic() {
        if (!SmartDashboard.containsKey("Aimer Offset")) {
            SmartDashboard.putNumber("Aimer Offset", ScoringConstants.aimerStaticOffset);
        }

        if (state == ScoringState.TEMPORARY_SETPOINT) {
            aimerIo.setAngleClampsRad(
                    ScoringConstants.aimMinAngleRadians, ScoringConstants.aimMaxAngleRadians);
        } else if (state != ScoringState.TUNING
                && state != ScoringState.ENDGAME
                && state != ScoringState.IDLE
                // && Math.abs(elevatorPositionSupplier.getAsDouble()) < 0.2
                && !overrideStageAvoidance
                && willHitStage()) {
            aimerIo.setAngleClampsRad(ScoringConstants.aimMinAngleRadians, 0);
        } else {
            double elevatorLimit =
                    aimerAvoidElevator.getValue(elevatorPositionSupplier.getAsDouble());
            Logger.recordOutput("scoring/elevatorPosition", elevatorPositionSupplier.getAsDouble());
            Logger.recordOutput("scoring/elevatorLimit", elevatorLimit);
            aimerIo.setAngleClampsRad(
                    Math.max(ScoringConstants.aimMinAngleRadians, elevatorLimit),
                    ScoringConstants.aimMaxAngleRadians);
        }

        aimerIo.controlAimAngleRad();

        aimerIo.setOverrideMode(state == ScoringState.OVERRIDE);
        hoodIo.setOverrideMode(state == ScoringState.OVERRIDE);
        shooterIo.setOverrideMode(state == ScoringState.OVERRIDE);

        Logger.recordOutput("scoring/State", state.toString());
        Logger.recordOutput("scoring/Action", action.toString());

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

        Logger.recordOutput("scoring/readyToShoot", readyToShoot);
        Logger.recordOutput("scoring/overrideShoot", overrideShoot);
        Logger.recordOutput("scoring/overrideStageAvoidance", overrideStageAvoidance);

        Logger.recordOutput("aimer/willIHitStage", willHitStage());

        Logger.recordOutput("scoring/distance", findDistanceToGoal());

        if (Constants.currentMode == Mode.SIM) {
            aimMechanism.setAngle(Units.radiansToDegrees(aimerInputs.aimAngleRad));
            hoodMechanism.setAngle(Units.radiansToDegrees(hoodInputs.hoodAngleRad));
            Logger.recordOutput("scoring/mechanism2d", mechanism);
        }

        switch (state) {
            case IDLE:
                idle();
                break;
            case INTAKE:
                intake();
                break;
            case SOURCE_INTAKE:
                sourceIntake();
                break;
            case SPIT:
                spit();
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
            case TUNING:
                tuning();
                break;
            case OVERRIDE:
                override();
                break;
            case TEMPORARY_SETPOINT:
                temporarySetpoint();
                break;
        }

        // If the robot is disabled, the pid should not be winding up
        if (DriverStation.isDisabled()) {
            aimerIo.resetPID();
            aimerIo.setAimAngleRad(aimerInputs.aimAngleRad, true);
        }

        shooterIo.updateInputs(shooterInputs);
        aimerIo.updateInputs(aimerInputs);
        hoodIo.updateInputs(hoodInputs);

        Logger.processInputs("scoring/shooter", shooterInputs);
        Logger.processInputs("scoring/aimer", aimerInputs);
        Logger.processInputs("scoring/hood", hoodInputs);
    }

    public void setTuningKickerVolts(double kickerVoltsTuning) {
        this.kickerVoltsTuning = kickerVoltsTuning;
    }

    public void setOverrideIntake(boolean overrideIntake) {
        this.overrideIntake = overrideIntake;
    }

    public void setOverrideShoot(boolean overrideShoot) {
        this.overrideShoot = overrideShoot;
    }

    public void setOverrideStageAvoidance(boolean overrideStageAvoidance) {
        this.overrideStageAvoidance = overrideStageAvoidance;
    }

    public void setOverrideBeamBrake(boolean overrideBeamBrake) {
        this.overrideBeamBreak = overrideBeamBrake;
    }

    public void setArmDisabled(boolean disabled) {
        aimerIo.setMotorDisabled(disabled);
    }

    public void forceHood(boolean hoodForced) {
        this.hoodForced = hoodForced;
    }

    public double getAimerAngle(double distance) {
        return aimerInterpolated.getValue(distance)
                + SmartDashboard.getNumber("Aimer Offset", ScoringConstants.aimerStaticOffset);
    }

    @Override
    public double getPosition(int slot) {
        switch (slot) {
                // Aimer
            case 0:
                return aimerInputs.aimAngleRad;
                // Hood
            case 1:
                return hoodInputs.hoodAngleRad;
                // Shooter
            case 2:
                return shooterInputs.shooterLeftVelocityRPM;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public double getVelocity(int slot) {
        switch (slot) {
                // Aimer
            case 0:
                return aimerInputs.aimVelocityRadPerSec;
                // Hood
            case 1:
                return hoodInputs.hoodVelocityRadPerSec;
                // Shooter
            case 2:
                return shooterInputs.shooterLeftVelocityRPM;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public double getConversionFactor(int slot) {
        switch (slot) {
                // Aimer
            case 0:
                return 1.0;
                // Hood
            case 1:
                return Math.PI;
                // Shooter
            case 2:
                return 1.0;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public void setVolts(double volts, int slot) {
        switch (slot) {
                // Aimer
            case 0:
                aimerIo.setOverrideVolts(volts);
                break;
                // Hood
            case 1:
                hoodIo.setOverrideVolts(volts);
                break;
                // Shooter
            case 2:
                shooterIo.setOverrideVolts(volts);
                break;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public void setPID(double p, double i, double d, int slot) {
        switch (slot) {
                // Aimer
            case 0:
                aimerIo.setPID(p, i, d);
                break;
                // Hood
            case 1:
                hoodIo.setPID(p, i, d);
                break;
                // Shooter
            case 2:
                shooterIo.setPID(p, i, d);
                break;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public void setMaxProfileProperties(double maxVelocity, double maxAcceleration, int slot) {
        switch (slot) {
                // Aimer
            case 0:
                aimerIo.setMaxProfile(maxVelocity, maxAcceleration);
                break;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public void setFF(double kS, double kV, double kA, double kG, int slot) {
        switch (slot) {
                // Aimer
            case 0:
                aimerIo.setFF(kS, kV, kA, kG);
                break;
                // Shooter
            case 2:
                shooterIo.setFF(kS, kV, kA);
                break;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public void runToPosition(double position, int slot) {
        state = ScoringState.TEMPORARY_SETPOINT;

        temporarySetpointPosition = position * getConversionFactor(slot);
        temporarySetpointSlot = slot;

        switch (slot) {
                // Aimer
            case 0:
                aimerIo.setAimAngleRad(temporarySetpointPosition, true);
                break;
                // Hood
            case 1:
                hoodIo.setHoodAngleRad(temporarySetpointPosition);
                break;
                // Shooter
            case 2:
                shooterIo.setShooterVelocityRPM(temporarySetpointPosition);
                break;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
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

    public void setbrakeMode(boolean brake) {
        aimerIo.setBrakeMode(brake);
        hoodIo.setBrakeMode(brake);
    }

    public void setAimerStatorCurrentLimit(double limit) {
        aimerIo.setStatorCurrentLimit(limit);
    }
}
