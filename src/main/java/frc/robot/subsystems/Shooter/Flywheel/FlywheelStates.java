package frc.robot.subsystems.Shooter.Flywheel;

public enum FlywheelStates {
    OFF(0),
    SAFE(0),
    FERRY(0),
    BUMP(0),
    SPIT(8), // todo: find out what this should be
    HUB(10),
    SPITTOCONTAINER(3),
    FIXEDFIRE(26),
    REVERSEFLYWHEEL(-10);

    public double FlywheelRightRPS;

    private FlywheelStates(double flywheelRPS) {
        this.FlywheelRightRPS = flywheelRPS;
    }
}
