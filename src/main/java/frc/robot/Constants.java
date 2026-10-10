// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.RobotTypes;
import frc.robot.subsystems.Intake.IntakeStates;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  //variable constants
  public static double intakePosition = IntakeStates.EXTENDED.intakePosition;
  // vision
  // public static final Transform3d tranformCenterRobotToAlgaeCamera = new Transform3d(0.35,0.145,0.15,new Rotation3d(0,0,Math.toRadians(30))); //TODO piecevis
  public static final double VisionSubStateAllignTollerance = 0.1;
  public static final double VisionAllignTollerance = 0.05;
  public static final double VisionAllignRotTolleranceToPerportinalSpeed = 0.5;
  public static final double VisionAllignRotationTollerance = 0.2;
  public static final double VisionAllignspeed = 1;
  public static final double VisionAllignRotspeed = 3;
  public static final double FlywheelDiamiter = 4*0.0254;
  public static final double TurretDistFromCenter = 0.21841;
  public static final double TurretAngleFromCenter = -2.1524498;
  public static final String pieceVisionCameraName = "pieceVisionCamera";

  public static final String frontRightCameraName = "FrontRightCamera";
  public static final String frontleftCameraName = "FrontLeftCamera";
  public static final String rightCameraName = "RightCamera";
  public static final String leftCameraName = "LeftCamera";
  public static final String backCameraName = "BackCamera";

  // public static final String intakeCameraName = "IntakeCamera";
  public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  private static final double highCamPitch = Units.degreesToRadians(20);
  private static final double LowCamPitch = Units.degreesToRadians(28.5);
  //-20 -26.57
  public static final Transform3d robotToCamFrontRight = new Transform3d(new Translation3d(-0.28,0.195,0.41),
      new Rotation3d(0, -highCamPitch, (-30/180.0)*Math.PI+Math.PI));
  //-20 +30
//
  public static Transform3d centerToCameraDefaultPosition = new Transform3d(0.25,0,0.3125, new Rotation3d(Units.degreesToRadians(0.66), Units.degreesToRadians(-9.5),0));//i think this is what inside robot pos is :) (7/10/26)
  //PIECEVISION PIECE VISION //i think its x 0 y 12.5in to end of hopper (need to remove part of crossbeam), idk z, prob keep rot at 0 -0.0380249839496*5
//  z=0 x=pos//z=up pos x, no y //0.3125
//extended = x=46.5, z=26.5

  public static final Transform3d robotToCamFrontLeft = new Transform3d(new Translation3d(-0.28,-0.135,0.24),
      new Rotation3d(0, -LowCamPitch, (30/180.0)*Math.PI + Math.PI));
  public static final Transform3d robotToCamRight = new Transform3d(new Translation3d(-0.265, 0.36, .465),
      new Rotation3d(0, -highCamPitch, Math.PI/2));
  public static final Transform3d robotToCamLeft = new Transform3d(new Translation3d(-0.27,-0.36, 0.245),
      new Rotation3d(0, -LowCamPitch, 3.0*Math.PI/2.0));
  public static final Transform3d robotToCamBack = new Transform3d(new Translation3d(-0.28,0.09, 0.42), //set these values
      new Rotation3d(0, -highCamPitch,0));  //and() these
  // The standard deviations of our vision estimated poses, which affect
  // correction rate    
  // (Fake values. Experiment anl;y   y y    +        u'  d determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  public static final Matrix<N3, N1> kMultiTagHubStdDevs = VecBuilder.fill(0.3, 0.3, 0.6);

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static boolean isBlueAlliance = true;
  public static Timer timer = new Timer();
  public static int NUM_LEDS;

  public static final double maxTelopVelocity = 1;// TODO
  public static final double maxTelopAngularVelocity = 1;// TODO
  public static final double leftYDeadband = 0.1;// TODO
  public static final double leftXDeadband = 0.1;// TODO
  public static final double rightXDeadband = 0.15;// TODO
  public static final double rightYDeadband = 0.15;
  public static final double maxAngularVelocity = 6;// TODO
  // public static final enum currentMode = 1;//TODO
  public static final double maxVelocity = 3.75;// TODO

  public static boolean isZeroed = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static int flyWheelRightID = 21;
  public static int flyWheelLeftID = 16;
  public static int hoodID = 13;
  public static int turretID = 17;
  public static int spindexerID = 26;
  public static int spindexerFollowerID = 27; //TODO remember to set the motor to 27
  public static int verticalFeederID = 18;
  public static int gateFeederID = 15;

  public static double turretZeroPosition = 200.25/360.0;
  public static double maximumHoodPosition = 77/360.0;
  public static double minimumHoodPosition = 58/360.0;
  public static double maximumTurretPosition = 0.8; // TODO set this to an actual value so sinjin doesnt cry
  public static double turretGearRatio = (60 / 14.0) * (156 / 20.0); //it is flipped to allign turret and gyro yaw rotation
  public static double hoodGearRatio = 522/21.0;

  public static double flywheelGearRatio = 1;//29/33.0;
  public static double flywheelToTopRollerRatio = 28/23.0;
  public static double flywheelRadius = 0.0508;
  public static double topRollerRadius = 0.0254;

  public static double feederGearRatio = 6.763;

  public static double gateGearRatio = 3.87;

  // public static double spindexerRadius = 3.25*0.0254;
  public static double spindexerRadius = 1;
  public static double spindexerGearRatio = 4.07;

  public static double rotateGearRatio = 12.0;// TODO fix this gear ratio

  public static double intakePivotGearRatio = 1.0/Math.PI; 
  public static double intakeRollerGearRatio = 2;
  public static int intakePivotFollowerID = 22;
  public static int intakeRollerFollowerID = 24;

  public static int intakePivotID = 19;
  public static int intakeRollerID = 23;

  public static int climbRotateControlledID = 14;

  public static RobotTypes robotType = RobotTypes.COMP;

  public static double redTrenchX = 11.7;//11.9?
  public static double blueTrenchX = 4.8;//4.6?
  public static double outpostTrenchY = 7.4375;
  public static double depotTrenchY = 0.625;

  

  //climb positions Red
    public static Translation2d climbStartPositionRedPosY = new Translation2d(15.500,6.106); //TODO get a real value
    public static Translation2d climbEngagedPositionRedPosY = new Translation2d(15.500,4.802); //TODO get a real value
    public static double climbStartRotationRedPosY = Math.PI/2; //TODO get a real value

    public static Translation2d climbStartPositionRedNegY = new Translation2d(15.500,2.296); //TODO get a real value
    public static Translation2d climbEngagedPositionRedNegY = new Translation2d(15.500,3.527); //TODO get a real value
    public static double climbStartRotationRedNegY = -Math.PI/2; //TODO get a real value
    //climb positions Blue
    public static Translation2d climbStartPositionBluePosY = new Translation2d(1.055,6.106); //TODO get a real value
    public static Translation2d climbEngagedPositionBluePosY = new Translation2d(1.055,4.802); //TODO get a real value
    public static double climbStartRotationBluePosY = Math.PI/2; //TODO get a real value

    public static Translation2d climbStartPositionBlueNegY = new Translation2d(1.055,2.296); //TODO get a real value
    public static Translation2d climbEngagedPositionBlueNegY = new Translation2d(1.055,3.527); //TODO get a real value
    public static double climbStartRotationBlueNegY = -Math.PI/2; //TODO get a real value

    public static double fieldCenterY = 4.02082;

    public static final double maxTurretAngle = (208/180.0)*Math.PI;
    public static final double minTurretAngle = -(274/180.0)*Math.PI;

  // set to ALPHA later
  public static enum RobotTypes {
    // alpha
    COMP, // swerve bot

    // beta
    BETA, // turret bot

    // swervebot
    SWERVE
  }

  public static CANBus krakenBus = new CANBus("krakenbus");
  public static Alliance allianceCOLOR;
}