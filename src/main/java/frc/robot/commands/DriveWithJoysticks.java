package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain.AlignTarget;
import frc.robot.utils.Deadband;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class DriveWithJoysticks extends Command {
    CommandSwerveDrivetrain drivetrain;
    DoubleSupplier x;
    DoubleSupplier y;
    DoubleSupplier rot;
    IntSupplier POV;
    BooleanSupplier fieldCentric;
    BooleanSupplier babyMode;
    BooleanSupplier toggleAlign;

    boolean lastToggleAlign;

    double xMpS;
    double yMpS;
    double rotRadpS;

    public DriveWithJoysticks(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rot,
            IntSupplier POV,
            BooleanSupplier fieldCentric,
            BooleanSupplier babyMode,
            BooleanSupplier toggleAlign) {
        this.drivetrain = drivetrain;
        this.x = x;
        this.y = y;
        this.POV = POV;
        this.rot = rot;
        this.fieldCentric = fieldCentric;
        this.babyMode = babyMode;
        this.toggleAlign = toggleAlign;

        this.lastToggleAlign = toggleAlign.getAsBoolean();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double[] joystickInputsFiltered =
                Deadband.twoAxisDeadband(
                        x.getAsDouble(), y.getAsDouble(), DriveConstants.deadbandPercent);

        xMpS = joystickInputsFiltered[0] * DriveConstants.MaxSpeedMetPerSec;
        yMpS = joystickInputsFiltered[1] * DriveConstants.MaxSpeedMetPerSec;
        rotRadpS =
                Deadband.oneAxisDeadband(rot.getAsDouble(), DriveConstants.deadbandPercent)
                        * DriveConstants.MaxAngularRateRadiansPerSec;

        if (babyMode.getAsBoolean()) {
            xMpS *= 0.5;
            yMpS *= 0.5;
            rotRadpS *= 0.5;
        }

        AlignTarget target = AlignTarget.NONE;
        switch (POV.getAsInt()) {
            case 0:
                target = AlignTarget.SPEAKER;
                break;
            case 90:
                target = AlignTarget.AMP;
                break;
            case 180:
                target = AlignTarget.NONE;
                break;
            case 270:
                target = AlignTarget.SOURCE;
                break;
            default:
                break;
        }

        drivetrain.setGoalChassisSpeeds(
                new ChassisSpeeds(xMpS, yMpS, rotRadpS),
                fieldCentric.getAsBoolean(),
                POV.getAsInt(),
                toggleAlign.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
