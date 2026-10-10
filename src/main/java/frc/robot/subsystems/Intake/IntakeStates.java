package frc.robot.subsystems.Intake;

   /**
   * The premade roller speed and intake positions for {@link frc.robot.subsystems.Intake.Intake Intake}
   * <br></br>
   * <b>Roller RPS and Position</b>
   * <ul>
   * <li>INTAKING - 45 - 0.34</li>
   * <li>EXTENDED - 0 - 0.34</li>
   * <li>PARTIALEXTENTION - 45 - 0.28</li>
   * <li>SAFE - 0 - 0</li>
   * <li>REVERSEINTAKING - (-30) - 0.3</li>
   * <li>ZERO - 0 - 0</li>
   * <li>ELEPHANTIASISPART2 - 45 - 0.34</li>
   * </ul>
   */

public enum IntakeStates {
    INTAKING(40, 12),//0,3.65
    AUTOINTAKING(55, 12),
    EXTENDED(0, 12),
    PARTIALEXTENTION(15,6),//was 5
    LESSPARTIALEXTENTION(15,10),
    SEMIPARTIALEXTENTION(15,8),
    SAFE(0, 2),
    REVERSEINTAKING(-35, 12),
    ZERO(0, 0),
    ELEPHANTIASISPART2(25,6)
    ,BEYONDMAX(0, 12)
    ,ZEROBUTITDOESNTBREAK(0,0);

    //LINTAKE PIVOT MAX: 0 MIN 0.76
   /**
   * The roller speed
   */
    public final double rollerRPS;

    /**
   * The intake position, like inside or outside the robot
   */
    public final double intakePosition;

    private IntakeStates(double rollerRPS, double intakePosition) {

        this.rollerRPS = rollerRPS;
        this.intakePosition = intakePosition;
    }
}

