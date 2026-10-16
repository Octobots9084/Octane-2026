package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.Vision.VisionIOSystem.EstimateConsumer;

//implement directional 45deg rot lock for intake

public class Vision extends SubsystemBase{
    public static enum VisionStates {
        BUMPING,
        CLIMB,
        SHOOTINGINHUB,
        FERRYING
    }

    private static Vision instance; 
    public final VisionIO io;
    private VisionStates visionState = VisionStates.SHOOTINGINHUB;
    private VisionStates visionWantedState = VisionStates.SHOOTINGINHUB;
    private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

    public static Vision getInstance(){
        return instance;
    }

    public Vision(EstimateConsumer estConsumer) {
        io = new VisionIOSystem(estConsumer);
        instance = this;
    }

    public void periodic() {
        io.periodic();
        io.updateInputs(visionInputs);
        Logger.processInputs("Vision", visionInputs);
        ApplyStates();
        handleStateTransitions();
    }

    public void ApplyStates() {
        switch (visionState) {
            case SHOOTINGINHUB:
                break;
            case BUMPING:
                break;
            case FERRYING:
                break;
            default:
                break;
        }
    }

    public void handleStateTransitions() {
        switch (visionWantedState) {
            case SHOOTINGINHUB:
                visionState = VisionStates.SHOOTINGINHUB;
                break;
            case BUMPING:
                visionState = VisionStates.BUMPING;
                break;
            case FERRYING:
                visionState = VisionStates.FERRYING;
                break;
            default:
                break;
        }
    }
}
