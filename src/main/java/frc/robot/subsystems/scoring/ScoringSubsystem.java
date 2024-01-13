package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class ScoringSubsystem extends SubsystemBase {
    private final ShooterIO shooterIo;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final AimerIO aimerIo;
    private final AimerIOInputsAutoLogged aimerInputs = new AimerIOInputsAutoLogged();

    private enum ScoringState {
        IDLE,
        PRIMING,
        INTAKING,
        SHOOTING,
        ENDGAME
    }

    private enum RequestedState {
        NONE,
        INTAKE,
        SHOOT,
        ENDGAME
    }

    private ScoringState state = ScoringState.IDLE;
    private RequestedState requestedState = RequestedState.NONE;

    public ScoringSubsystem(ShooterIO shooterIo, AimerIO aimerIo) {
        this.shooterIo = shooterIo;
        this.aimerIo = aimerIo;
    }

    public void request(RequestedState requestedState) {
        this.requestedState = requestedState;
    }

    

    @Override
    public void periodic() {
        switch (state) {
            case IDLE:
                if(requestedState == RequestedState.INTAKE)
                {
                    state = ScoringState.INTAKING;
                }
                else if(requestedState == RequestedState.SHOOT || requestedState)
                {
                    state = ScoringState.AIMING;
                }
                else if(requestedState == RequestedState.ENDGAME)
                {
                    state = ScoringState.ENDGAME;
                }
                break;
            case PRIMING:
                if(requestedState == RequestedState.SHOOT)
                {
                    // X
                }
                else if(requestedState == RequestedState.INTAKE)
                {
                    // X
                }
                else if(requestedState == RequestedState.ENDGAME)
                {
                    // X
                }
                break;
            case INTAKING:
                aimerIo.setAimAngRad(0);
                shooterIo.setShooterVelRPM(-5);
                if(requestedState != RequestedState.INTAKE)
                {
                    state = ScoringState.IDLE;
                }
                break;
            case SHOOTING:
                break;
            case ENDGAME:
                break;
        }
    }
}
