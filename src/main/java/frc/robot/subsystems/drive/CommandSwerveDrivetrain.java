package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private double vx, vy, omega;
    private boolean fieldCentric = true; 

    private SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();
    private SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric();
    private SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
    }

    public void setGoalChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean fieldCen) {
        vx = chassisSpeeds.vxMetersPerSecond;
        vy = chassisSpeeds.vyMetersPerSecond;
        omega = chassisSpeeds.omegaRadiansPerSecond;
        fieldCentric = fieldCen;

    }

    private void controlDrivetrain() {
        if (vx == 0 && vy == 0 && omega == 0) {
            setControl(brake);
        } else {
            if (!fieldCentric) {
                setControl(driveRobotCentric
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(omega));
            } else {
                setControl(driveFieldCentric
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(omega));
            }
        }
    }

    @Override
    public void periodic() {
        if (Constants.currentMode == Constants.Mode.SIM) {
            updateSimState(0.02, 12);
        }
        SmartDashboard.putNumber("x", vx);

        controlDrivetrain();
        
    }
}