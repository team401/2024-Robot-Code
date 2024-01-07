package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.Deadband;

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

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true);
    SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric().withIsOpenLoop(true);
    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public DriveWithJoysticks(CommandSwerveDrivetrain drivetrain, DoubleSupplier x, DoubleSupplier y,
            DoubleSupplier rot, BooleanSupplier fieldCentric, BooleanSupplier babyMode) {
        this.drivetrain = drivetrain;
        this.x = x;
        this.y = y;
        this.rot = rot;
        this.fieldCentric = fieldCentric;
        this.babyMode = babyMode;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double[] joystickInputsFiltered = Deadband.twoAxisDeadband(x.getAsDouble(), y.getAsDouble(),
                DriveConstants.deadbandPercent);

        xMpS = joystickInputsFiltered[0] * DriveConstants.MaxSpeedMetPerSec;
        yMpS = joystickInputsFiltered[1] * DriveConstants.MaxSpeedMetPerSec;
        rotRadpS = Deadband.oneAxisDeadband(rot.getAsDouble(), DriveConstants.deadbandPercent)
                * DriveConstants.MaxAngularRateRadiansPerSec;

        if (babyMode.getAsBoolean()) {
            xMpS *= 0.5;
            yMpS *= 0.5;
            rotRadpS *= 0.5;
        }

        if (xMpS == 0 && yMpS == 0 && rotRadpS == 0) {
            // If not moving, brake
            drivetrain.applyRequest(() -> brake);
        } else if (fieldCentric.getAsBoolean()) {
            drivetrain.applyRequest(() -> drive.withVelocityX(-xMpS)
                    .withVelocityY(-yMpS)
                    .withRotationalRate(-rotRadpS));
        } else {
            drivetrain.applyRequest(() -> driveRobot.withVelocityX(-xMpS)
                    .withVelocityY(-yMpS)
                    .withRotationalRate(-rotRadpS));
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}