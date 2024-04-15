package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import java.util.function.Supplier;

public class LED extends SubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private Timer timer;

    private boolean enabled = true;

    private final int ledcount = LEDConstants.ledLength;
    private final int ledBrightness = 100;

    private int offset = 0;
    private final int rainbowSpeed = 4;
    private final int rainbowScale = 2;

    private ScoringSubsystem scoringSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private Supplier<Boolean> visionWorkingSupplier = () -> true;

    public LED(ScoringSubsystem scoringSubsystem, IntakeSubsystem intakeSubsystem) {
        led = new AddressableLED(LEDConstants.ledPort);
        ledBuffer = new AddressableLEDBuffer(ledcount);
        led.setLength(ledBuffer.getLength());
        timer = new Timer();
        this.scoringSubsystem = scoringSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        led.setData(ledBuffer);
        led.start();
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public void periodic() {

        clear();

        if (!enabled) {
            // LEDs left cleared if not enabled
        } else if (DriverStation.isDisabled()) {
            if (Constants.currentMode == Mode.REAL) {
                rainbow();

                if (!visionWorkingSupplier.get()) {
                    for (int i = 0; i < ledcount - 10; i++) {
                        ledBuffer.setRGB(i, 0, 0, 255 / 3);
                    }
                }
            }

        } else {
            if ((scoringSubsystem != null && scoringSubsystem.hasNote())
                    || (intakeSubsystem != null && intakeSubsystem.hasNote())) {
                for (int i = 0; i < ledcount; i++) {
                    ledBuffer.setRGB(i, 245 / 3, 117 / 3, 66 / 3);
                }
            }

            if (DriverStation.getMatchTime() < 20 && DriverStation.getMatchTime() > 17) {
                for (int i = 0; i < ledcount; i++) {
                    ledBuffer.setRGB(i, 255 / 3, 0, 0);
                }
            }

            if (!visionWorkingSupplier.get()) {
                for (int i = 0; i < ledcount - 10; i++) {
                    ledBuffer.setRGB(i, 0, 0, 255 / 3);
                }
            }

            // // idle
            // if (scoringSubsystem.getCurrentState() == ScoringState.IDLE) {
            //     if (Constants.currentMode == Mode.REAL)
            //         setFlashingColorSection(
            //                 0, ledcount, new int[] {105, 29, 16}, new int[] {255, 123, 0});
            //     else SmartDashboard.putString("current task", "idle");
            // }

            // // intake
            // if (scoringSubsystem.getCurrentState() == ScoringState.INTAKE) {
            //     if (scoringSubsystem.hasNote()) {
            //         if (Constants.currentMode == Mode.REAL)
            //             setSolidColorSection(0, ledcount, new int[] {255, 174, 0});
            //         else SmartDashboard.putString("current task", "intake, has note");
            //     } else {
            //         if (Constants.currentMode == Mode.REAL)
            //             setFlashingColorSection(
            //                     0, ledcount, new int[] {255, 174, 0}, new int[] {0, 0, 0});
            //         else SmartDashboard.putString("current task", "intake");
            //     }
            // }

            // // shooting amp
            // if (scoringSubsystem.getCurrentState() == ScoringState.AMP_PRIME) {
            //     if (scoringSubsystem.readyToShoot()) {
            //         if (Constants.currentMode == Mode.REAL)
            //             setSolidColorSection(0, ledcount, new int[] {32, 227, 64});
            //         else SmartDashboard.putString("current task", "amp, ready to shoot");
            //     } else {
            //         if (Constants.currentMode == Mode.REAL)
            //             setFlashingColorSection(
            //                     0, ledcount, new int[] {32, 227, 64}, new int[] {0, 0, 0});
            //         else SmartDashboard.putString("current task", "amp, preparing");
            //     }
            // }

            // // shooting speaker
            // if (scoringSubsystem.getCurrentState() == ScoringState.PRIME) {
            //     int[] rgbCode = new int[3];
            //     if (DriverStation.getAlliance().isEmpty()
            //             || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            //         rgbCode[0] = 0;
            //         rgbCode[1] = 30;
            //         rgbCode[2] = 225;
            //     } else {
            //         rgbCode[0] = 225;
            //         rgbCode[1] = 0;
            //         rgbCode[2] = 0;
            //     }

            //     if (scoringSubsystem.readyToShoot()) {
            //         if (Constants.currentMode == Mode.REAL)
            //             setSolidColorSection(0, ledcount, rgbCode);
            //         else SmartDashboard.putString("current task", "speaker, ready to shoot");
            //     } else {
            //         if (Constants.currentMode == Mode.REAL)
            //             setFlashingColorSection(0, ledcount, rgbCode, new int[] {0, 0, 0});
            //         else SmartDashboard.putString("current task", "speaker, preparing");
            //     }
            // }

            // // endgame
            // if (scoringSubsystem.getCurrentState() == ScoringState.ENDGAME) {
            //     if (Constants.currentMode == Mode.REAL)
            //         setSolidColorSection(0, ledcount, new int[] {140, 0, 255});
            //     else SmartDashboard.putString("current task", "endgame");
            // }
        }

        if (Constants.currentMode == Mode.REAL) {
            led.setData(ledBuffer);
        }
    }

    public void setVisionWorkingSupplier(Supplier<Boolean> visionWorkingSupplier) {
        this.visionWorkingSupplier = visionWorkingSupplier;
    }

    private void clear() {
        for (int i = 0; i < ledcount; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    private void setSolidColorSection(int lower, int upper, int[] rgbCode) {
        for (int i = lower; i <= upper; i++) {
            ledBuffer.setRGB(i, rgbCode[0], rgbCode[1], rgbCode[2]);
        }
    }

    private void setFlashingColorSection(int lower, int upper, int[] rgbCode1, int[] rgbCode2) {
        if ((Math.floor(timer.get()) / 5.0) % 2 == 0) {
            for (int i = lower; i <= upper; i++) {
                ledBuffer.setRGB(i, rgbCode1[0], rgbCode1[1], rgbCode1[2]);
            }
        } else {
            for (int i = lower; i <= upper; i++) {
                ledBuffer.setRGB(i, rgbCode2[0], rgbCode2[1], rgbCode2[2]);
            }
        }
    }

    private void rainbow() {
        for (int i = 0; i < ledcount; i++) {

            int hue = (offset + i * rainbowScale) % 180;
            ledBuffer.setHSV(i, hue, 255, ledBrightness);
        }

        offset += rainbowSpeed;
        offset %= 180;
    }
}
