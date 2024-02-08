package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringAction;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringState;

public class LED implements Subsystem {

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private Timer timer;

    private final int ledcount = 1000;

    private ScoringSubsystem scoringSubsystem;

    public LED(ScoringSubsystem scoringSubsystem) {
        led = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(ledcount);
        timer = new Timer();
        this.scoringSubsystem = scoringSubsystem;
    }


    @Override
    public void periodic() {

        clear();
        
        if (DriverStation.isDisabled()){
                rainbow();
            
        } else {
                //idle
                if (scoringSubsystem.getCurrentAction() == ScoringAction.WAIT && scoringSubsystem.getCurrentState() == ScoringState.IDLE){
                    setFlashingColorSection(0, ledcount, new int[] {105, 29, 16}, new int[] {255, 123, 0});
                    SmartDashboard.putString("current task", "idle");
                }

                //intake
                if (scoringSubsystem.getCurrentState() == ScoringState.INTAKE) {
                    if (scoringSubsystem.hasNote()) {
                        setSolidColorSection(0, ledcount, new int[] {255, 174, 0});
                        SmartDashboard.putString("current task", "intake, has note");
                    } else {
                        setFlashingColorSection(0, ledcount, new int[] {255, 174, 0}, new int[] {0,0,0});
                        SmartDashboard.putString("current task", "intake");
                    }
                    
                }
                
                //shooting amp
                if (scoringSubsystem.getCurrentState() == ScoringState.AMP_PRIME) {
                    if (scoringSubsystem.readyToShoot) {
                        setSolidColorSection(0, ledcount, new int[] {32, 227, 64});
                        SmartDashboard.putString("current task", "amp, ready to shoot");
                    } else {
                        setFlashingColorSection(0, ledcount, new int[] {32, 227, 64}, new int[] {0,0,0});
                        SmartDashboard.putString("current task", "amp, preparing");
                    }
                }

                //shooting speaker
                if (scoringSubsystem.getCurrentState() == ScoringState.PRIME) {
                    int[] rgbCode = new int[3];
                    if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                        rgbCode[0] = 0; rgbCode[1] = 30; rgbCode[2] = 225;
                    } else {
                        rgbCode[0] = 225; rgbCode[1] = 0; rgbCode[2] = 0;
                    }

                    if (scoringSubsystem.readyToShoot) {
                        setSolidColorSection(0, ledcount, rgbCode);
                        SmartDashboard.putString("current task", "speaker, ready to shoot");
                    } else {
                        setFlashingColorSection(0, ledcount, rgbCode, new int[] {0,0,0});
                        SmartDashboard.putString("current task", "speaker, preparing");
                    }
                }

                //endgame
                if (scoringSubsystem.getCurrentState() == ScoringState.ENDGAME) {
                    setSolidColorSection(0, ledcount, new int[] {140, 0, 255});
                    SmartDashboard.putString("current task", "endgame");
                }
                
                }
        led.setData(ledBuffer);

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
            int hue = (90 + (i * 180 / ledcount)) % 180;
            ledBuffer.setHSV(i, hue, 255, 127); 
        }

    }
    
}
