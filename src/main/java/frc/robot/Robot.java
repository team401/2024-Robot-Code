// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    @SuppressWarnings("unused")
    private RobotContainer robotContainer;

    @SuppressWarnings("unused")
    private PowerDistribution pdh;

    public Robot() {
        super(Constants.loopTime);
    }

    @Override
    public void robotInit() {
        Logger.recordMetadata("ProjectName", "2024 - 401 Comp Robot"); // TODO: Name the robot!

        if (Constants.currentMode == Constants.Mode.REAL) {
            // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs") TODO:
            // Add back later
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            pdh = new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else if (Constants.currentMode == Constants.Mode.SIM) {
            // Logger.addDataReceiver(new WPILOGWriter("logs/")); TODO: Add back later
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope
            // (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(
                    new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save
            // outputs
            // to
            // a
            // new
            // log
        }

        Logger.start();

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        robotContainer.robotPeriodic();
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();

        robotContainer.enabledInit();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        robotContainer.enabledInit();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        robotContainer.testInit("tuning-speaker");
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
