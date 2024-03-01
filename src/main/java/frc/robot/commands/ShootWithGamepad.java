package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain.AlignTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringAction;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ShootWithGamepad extends Command {

    private BooleanSupplier driverShoot;
    private BooleanSupplier driverForceShoot;
    private BooleanSupplier masherShoot;
    private BooleanSupplier masherForceShoot;
    private BooleanSupplier warmup;
    private BooleanSupplier reverseIntake;
    private BooleanSupplier intake;

    private ScoringSubsystem scoring;

    private Supplier<AlignTarget> getDriveMode;

    public ShootWithGamepad(
            BooleanSupplier driverShoot,
            BooleanSupplier driverForceShoot,
            BooleanSupplier masherShoot,
            BooleanSupplier masherForceShoot,
            BooleanSupplier warmup,
            BooleanSupplier reverseIntake,
            BooleanSupplier intake,
            ScoringSubsystem scoring,
            Supplier<AlignTarget> getDriveMode) {
        this.driverShoot = driverShoot;
        this.driverForceShoot = driverForceShoot;
        this.masherShoot = masherShoot;
        this.masherForceShoot = masherForceShoot;
        this.warmup = warmup;
        this.reverseIntake = reverseIntake;
        this.intake = intake;

        this.scoring = scoring;
        this.getDriveMode = getDriveMode;

        addRequirements(scoring);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (warmup.getAsBoolean()) {
            switch (getDriveMode.get()) {
                case NONE:
                    scoring.setAction(ScoringAction.WAIT);
                    break;
                case SPEAKER:
                    scoring.setAction(ScoringAction.AIM);
                    break;
                case AMP:
                    scoring.setAction(ScoringAction.AMP_AIM);
                    break;
                case SOURCE:
                    scoring.setAction(ScoringAction.SOURCE_INTAKE);
                    break;
                case ENDGAME:
                    scoring.setAction(ScoringAction.ENDGAME);
                    break;
            }

            /*
             * If the scorer is commanded to shoot from a non-shootable state, it will start trying to
             * shoot at the speaker. We want the shoot button to do nothing in an un-shootable state.
             */
            if (getDriveMode.get() != AlignTarget.SOURCE
                    && getDriveMode.get() != AlignTarget.NONE) {
                boolean force = driverForceShoot.getAsBoolean() || masherForceShoot.getAsBoolean();
                scoring.setOverrideShoot(force);

                if (driverShoot.getAsBoolean() || masherShoot.getAsBoolean() || force) {
                    scoring.setAction(ScoringAction.SHOOT);
                }
            }
        } else if (reverseIntake.getAsBoolean()) {
            scoring.setAction(ScoringAction.SPIT);
        } else if (intake.getAsBoolean()) {
            scoring.setAction(ScoringAction.INTAKE);
        } else {
            scoring.setAction(ScoringAction.WAIT);
        }
    }
}
