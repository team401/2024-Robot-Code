package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class WheelRadiusCharacterization extends Command {
    private static final double characterizationSpeed = 0.8;
    private static final double driveRadius = TunerConstants.kModuleRadiusMeters;

    private final DoubleSupplier gyroYawRadsSupplier; // need to get pose of the robot
    private final CommandSwerveDrivetrain drivetrain;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

    private double omegaDirection = 1;

    private double[] startWheelPositions;

    private double currentEffectiveWheelRadius = 0.0;

    public WheelRadiusCharacterization(
            CommandSwerveDrivetrain drive, DoubleSupplier gyroYawRadsSupplier) {
        drivetrain = drive;
        this.gyroYawRadsSupplier = gyroYawRadsSupplier;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Reset
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        accumGyroYawRads = 0.0;

        startWheelPositions = drivetrain.getWheelRadiusCharacterizationPosition();

        omegaLimiter.reset(0);

        if (!SmartDashboard.containsKey("Effective Wheel Radius")) {
            SmartDashboard.putNumber("Effective Wheel Radius", 0.0);
        }
    }

    @Override
    public void execute() {
        // Run drive at velocity
        drivetrain.runWheelRadiusCharacterization(
                omegaLimiter.calculate(omegaDirection * characterizationSpeed));

        // Get yaw and wheel positions
        accumGyroYawRads +=
                MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        double averageWheelPosition = 0.0;
        double[] wheelPositions = drivetrain.getWheelRadiusCharacterizationPosition();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
        SmartDashboard.putNumber(
                "TestMode/RadiusCharacterization/DrivePosition", averageWheelPosition);
        SmartDashboard.putNumber(
                "TestMode/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
        SmartDashboard.putNumber(
                "TestMode/RadiusCharacterization/CurrentWheelRadiusInches",
                Units.metersToInches(currentEffectiveWheelRadius));
    }

    @Override
    public void end(boolean interrupted) {
        if (Math.abs(accumGyroYawRads) <= Math.PI * 2.0) {
            SmartDashboard.putNumber("Effective Wheel Radius", -1.0);
        } else {
            SmartDashboard.putNumber("Effective Wheel Radius", currentEffectiveWheelRadius);
        }

        drivetrain.setGoalChassisSpeeds(new ChassisSpeeds());
    }
}
