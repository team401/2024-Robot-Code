package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.utils.Deadband;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveWithJoysticks extends Command {
    CommandSwerveDrivetrain drivetrain;
    DoubleSupplier x;
    DoubleSupplier y;
    DoubleSupplier rot;
    BooleanSupplier fieldCentric;
    BooleanSupplier babyMode;

    double xMpS;
    double yMpS;
    double rotRadpS;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    public DriveWithJoysticks(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rot,
            BooleanSupplier fieldCentric,
            BooleanSupplier babyMode) {
        this.drivetrain = drivetrain;
        this.x = x;
        this.y = y;
        this.rot = rot;
        this.fieldCentric = fieldCentric;
        this.babyMode = babyMode;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double allianceX = 0.0;
        double allianceY = 0.0;
        if (DriverStation.getAlliance().isPresent()) {
            switch (DriverStation.getAlliance().get()) {
                case Red:
                    allianceX = x.getAsDouble();
                    allianceY = y.getAsDouble();
                    break;
                case Blue:
                    allianceX = -x.getAsDouble();
                    allianceY = -y.getAsDouble();
                    break;
            }
        } else {
            allianceX = x.getAsDouble();
            allianceY = y.getAsDouble();
        }
        double[] joystickInputsFiltered =
                Deadband.twoAxisDeadband(allianceX, allianceY, DriveConstants.deadbandPercent);

        joystickInputsFiltered[0] = Math.pow(joystickInputsFiltered[0], 1);
        joystickInputsFiltered[1] = Math.pow(joystickInputsFiltered[1], 1);

        xMpS = joystickInputsFiltered[0] * DriveConstants.MaxSpeedMetPerSec;
        yMpS = joystickInputsFiltered[1] * DriveConstants.MaxSpeedMetPerSec;
        rotRadpS = Deadband.oneAxisDeadband(rot.getAsDouble(), DriveConstants.deadbandPercent);
        rotRadpS = Math.pow(rotRadpS, 1) * DriveConstants.MaxAngularRateRadiansPerSec;

        if (babyMode.getAsBoolean()) {
            xMpS *= 0.5;
            yMpS *= 0.5;
            rotRadpS *= 0.5;
        }

        chassisSpeeds.vxMetersPerSecond = xMpS;
        chassisSpeeds.vyMetersPerSecond = yMpS;
        chassisSpeeds.omegaRadiansPerSecond = rotRadpS;
        drivetrain.setGoalChassisSpeeds(chassisSpeeds, fieldCentric.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
