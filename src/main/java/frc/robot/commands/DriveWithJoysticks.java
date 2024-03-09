package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.utils.Deadband;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveWithJoysticks extends Command {
    CommandSwerveDrivetrain drivetrain;
    DoubleSupplier x;
    DoubleSupplier y;
    DoubleSupplier rot;
    BooleanSupplier fieldCentric;
    BooleanSupplier babyMode;
    Supplier<Vector<N2>> currentVelocitySupplier;

    double commandedXMpS;
    double commandedYMpS;
    double commandedRotRadpS;

    double currentXMpS;
    double currentYMpS;

    double lastTime = Utils.getCurrentTimeSeconds();

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    public DriveWithJoysticks(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rot,
            BooleanSupplier fieldCentric,
            BooleanSupplier babyMode,
            Supplier<Vector<N2>> currentVelocitySupplier) {
        this.drivetrain = drivetrain;
        this.x = x;
        this.y = y;
        this.rot = rot;
        this.fieldCentric = fieldCentric;
        this.babyMode = babyMode;
        this.currentVelocitySupplier = currentVelocitySupplier;

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

        commandedXMpS = joystickInputsFiltered[0] * DriveConstants.MaxSpeedMetPerSec;
        commandedYMpS = joystickInputsFiltered[1] * DriveConstants.MaxSpeedMetPerSec;
        commandedRotRadpS =
                Deadband.oneAxisDeadband(rot.getAsDouble(), DriveConstants.deadbandPercent);
        commandedRotRadpS =
                Math.pow(commandedRotRadpS, 1) * DriveConstants.MaxAngularRateRadiansPerSec;

        if (babyMode.getAsBoolean()) {
            commandedXMpS *= 0.5;
            commandedYMpS *= 0.5;
            commandedRotRadpS *= 0.5;
        }

        currentXMpS = currentVelocitySupplier.get().get(0, 0);
        currentYMpS = currentVelocitySupplier.get().get(1, 0);

        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;

        if (Math.hypot(currentXMpS, currentYMpS) < Math.hypot(commandedXMpS, commandedYMpS)) {
            double velocityVectorTheta = Math.atan2(commandedYMpS, commandedXMpS);
            commandedXMpS =
                    currentXMpS
                            + Math.signum(commandedXMpS)
                                    * (DriveConstants.maxAccelerationMetersPerSecSquared
                                            * Math.cos(velocityVectorTheta)
                                            * diffTime);
            commandedYMpS =
                    currentYMpS
                            + Math.signum(commandedYMpS)
                                    * (DriveConstants.maxAccelerationMetersPerSecSquared
                                            * Math.sin(velocityVectorTheta)
                                            * diffTime);
        }
        chassisSpeeds = new ChassisSpeeds(commandedXMpS, commandedYMpS, commandedRotRadpS);
        drivetrain.setGoalChassisSpeeds(chassisSpeeds, fieldCentric.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
