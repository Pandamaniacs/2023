package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;
import java.util.List;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Robot extends TimedRobot {
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  private XboxController driverController = new XboxController(0);
  private XboxController shootController = new XboxController(1);
  private DifferentialDrive myRobot;

  private static final int leftLeadID = 1;
  private static final int leftFollow1ID = 2;
  private static final int leftFollow2ID = 3;
  private static final int rightLeadID = 5;
  private static final int rightFollow1ID = 6;
  private static final int rightFollow2ID = 7;
  private static final int intakeID = 10;

  private CANSparkMax leftLeader;
  private CANSparkMax leftFollow1;
  private CANSparkMax leftFollow2;
  private CANSparkMax rightLeader;
  private CANSparkMax rightFollow1;
  private CANSparkMax rightFollow2;
  private CANSparkMax intakeMotor;

  private final Timer timer = new Timer();

  LED leftLed;
  final int leftLedPort = 0;
  final int statusLedLength = 12;
  LED rightLed;
  final int rightLedPort = 1;
  LED directionLed;
  final int directionLedPort = 2;
  final int directionLedLength = 9;
  /**
   * ALWAYS ENSURE LEDS ARE MOUNTED WITH PWM CABLE ON THE TOP LEFT
   */

  int status = 0;
  boolean high = false;
  int[] search;
  boolean flash = false;
  /**
   * 0: not searching
   * 1: searching, no target
   * 2: lining up
   * 3: ready
   */

  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  Solenoid phSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  Solenoid phSolenoid2 = new Solenoid(PneumaticsModuleType.REVPH, 1);

  double kP_dist = 0.1;
  double kD_dist = 0.0;
  double tolerance_dist = .05;
  PIDController PID_dist = new PIDController(kP_dist, 0, kD_dist);
  double kP_angle = 0.1;
  double kD_angle = 0.0;
  double tolerance_angle = 5;
  PIDController PID_angle = new PIDController(kP_dist, 0, kD_dist);

  double degrees = Units.radiansToDegrees(0);

  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);// height of the camera from the ground
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);// height of target
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);// goal range
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);// Angle between horizontal and the camera.

  DigitalInput back = new DigitalInput(0);
  DigitalInput shoot = new DigitalInput(1);
  DigitalInput intake = new DigitalInput(2);

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("kP_dist", kP_dist);
    SmartDashboard.putNumber("kD_dist", kD_dist);
    SmartDashboard.putNumber("kP_angle", kP_angle);
    SmartDashboard.putNumber("kD_angle", kD_angle);
    PID_dist.setTolerance(tolerance_dist, 10);
    PID_angle.setTolerance(tolerance_angle, 10);

    camera.setDriverMode(false);
    camera.setPipelineIndex(0);

    initMotors();
    initLEDS();

    myRobot = new DifferentialDrive(leftLeader, rightLeader);
    phCompressor.enableDigital();

    addPeriodic(() -> {
      updateLedPattern();
    }, 0.01, 0.005);
  }

  int first = 0;

  /**
   * 0: not searching - blank
   * 1: searching, no target - searching
   * 2: lining up - flash
   * 3: ready - red/white, low/high
   */
  
  /**
   * 3x4 led map
   * 0 1 2 3
   * 7 6 5 4
   * 8 9 10 11
   */
  /**
   * 3x3 led map
   * 0 1 2
   * 5 4 3
   * 6 7 8
   */
  
  int[] statusMap = {0,  1,  2,  3,
                     7,  6,  5,  4,
                     8,  9,  10, 11};
  int[] searchMap = {0, 1, 2, 3, 4, 11, 10, 9, 8, 7};
  int[] searchOutput = new int[searchMap.length];
  int[] dirMap = {0,  1,  2,
                  5,  4,  3,
                  6,  7,  8};
  public void updateLedPattern() {
    switch(status) {
      case 0:
        leftLed.clear();
        rightLed.clear();
        break;

      case 1:
      first = search[0];
        for(int i = 0 ; i < search.length - 1 ; i++) {
          search[i] = search[i + 1];
        }
        Arrays.fill(searchOutput, 0);
        for(int i = 0 ; i < searchMap.length ; i++) {
          for(int j = 0 ; j < statusMap.length ; i++) {
            if(statusMap[j] == searchMap[i]) {
              searchOutput[statusMap[j]] = search[i];
              break;
            }
          }
        }
        leftLed.setPatternSingleColor(searchOutput);
        search[search.length - 1] = first;
        break;

      case 2:
        flash = !flash;
        if(flash) {
          leftLed.setSolid(255, 0, 0);
          rightLed.setSolid(255, 255, 255);
        } else {
          leftLed.setSolid(255, 255, 255);
          rightLed.setSolid(255, 0, 0);
        }
        break;

      case 3:
        if(high) {
          leftLed.setSolid(255, 255, 255);
          rightLed.setSolid(255, 255, 255);
        } else {
          leftLed.setSolid(255, 0, 0);
          rightLed.setSolid(255, 0, 0);
        }
        break;
    }
  }

  public void initMotors() {
    leftLeader = new CANSparkMax(leftLeadID, MotorType.kBrushless);
    leftFollow1 = new CANSparkMax(leftFollow1ID, MotorType.kBrushless);
    rightLeader = new CANSparkMax(rightLeadID, MotorType.kBrushless);
    rightFollow1 = new CANSparkMax(rightFollow1ID, MotorType.kBrushless);
    leftFollow2 = new CANSparkMax(leftFollow2ID, MotorType.kBrushless);
    rightFollow2 = new CANSparkMax(rightFollow2ID, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);

    leftLeader.restoreFactoryDefaults();
    leftFollow1.restoreFactoryDefaults();
    leftFollow2.restoreFactoryDefaults();
    rightLeader.restoreFactoryDefaults();
    rightFollow1.restoreFactoryDefaults();
    rightFollow2.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();

    int driveCurrentLimit = 60;
    int intakeCurrentLimit = 30;
    leftLeader.setSmartCurrentLimit(driveCurrentLimit);
    leftFollow1.setSmartCurrentLimit(driveCurrentLimit);
    leftFollow2.setSmartCurrentLimit(driveCurrentLimit);
    rightLeader.setSmartCurrentLimit(driveCurrentLimit);
    rightFollow1.setSmartCurrentLimit(driveCurrentLimit);
    rightFollow2.setSmartCurrentLimit(driveCurrentLimit);
    intakeMotor.setSmartCurrentLimit(intakeCurrentLimit);

    leftFollow1.follow(leftLeader);
    leftFollow2.follow(leftLeader);
    rightFollow1.follow(rightLeader);
    rightFollow2.follow(rightFollow1);

    leftLeader.burnFlash();
    leftFollow1.burnFlash();
    leftFollow2.burnFlash();
    rightLeader.burnFlash();
    rightFollow1.burnFlash();
    rightFollow2.burnFlash();
    intakeMotor.burnFlash();
  }

  public void initLEDS() {
    leftLed = new LED(leftLedPort, statusLedLength);
    rightLed = new LED(rightLedPort, statusLedLength);
    directionLed = new LED(directionLedPort, directionLedLength);
    
    search = new int[10];

    for (int i = 0 ; i < search.length ; i++) {
      search[i] = 255/search.length*i;
    }
  }

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    setLed(0, 0, 255);
  }

  @Override
  public void autonomousPeriodic() {
    double speed = 0.75;
    if (back.get()) {
      if (timer.get() < 2.0) {
        leftLeader.set(speed);
        rightLeader.set(speed * -1);
      }
    } else if (shoot.get()) {
      if (timer.get() < 5.0) {
        phSolenoid.set(true);
        Timer.delay(.50);
        phSolenoid.set(false);
        leftLeader.set(speed * -1);
        rightLeader.set(speed);
      }
    } else if (intake.get()) {
      if (timer.get() < 4.0) {
        phSolenoid.set(true);
        Timer.delay(.50);
        phSolenoid.set(false);
        leftLeader.set(speed * -1);
        rightLeader.set(speed);
        // intake code added here :)
      }
    } else {
      myRobot.stopMotor();
    }
  }

  double forward = 0;
  double steer = 0;
  double p_dist = 0;
  double d_dist = 0;
  double p_angle = 0;
  double d_angle = 0;

  boolean distAtSet = false;
  boolean angleAtSet = false;
  boolean ready = false;

  @Override
  public void teleopPeriodic() {
    handleDash();

    double leftYstick = (driverController.getLeftY());
    double rightXstick = (driverController.getRightX());
    forward = leftYstick;
    steer = rightXstick;

    if (driverController.getBButtonPressed()) {
      phSolenoid.set(true);
      Timer.delay(1.0);
    } else if (driverController.getBButtonReleased()) {
      phSolenoid.set(false);
    }

    if (driverController.getXButtonPressed()) {
      phCompressor.disable();
    }

    if (driverController.getAButton()) {
      var result = camera.getLatestResult();

      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = targets.get(0);
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(result.getBestTarget().getPitch()));

        forward = -PID_dist.calculate(range, 30);
        steer = -PID_angle.calculate(result.getBestTarget().getYaw(), 0);

        distAtSet = PID_dist.atSetpoint();
        angleAtSet = PID_angle.atSetpoint();
        ready = distAtSet && angleAtSet;
        SmartDashboard.putBoolean("has target", true);// we see a target!
        SmartDashboard.putBoolean("distance good", distAtSet);// are we at a good distance?
        SmartDashboard.putBoolean("angle good", angleAtSet);// are we at a good angle?
        SmartDashboard.putBoolean("ready", ready);// are we at a good distance and angle?

        if ((PID_dist.atSetpoint() & PID_angle.atSetpoint())) {// yes!
          driverController.setRumble(RumbleType.kLeftRumble, .1);// let the drivers know with some rumble!
          shootController.setRumble(RumbleType.kLeftRumble, .1);
        }
        else {// no
          driverController.setRumble(RumbleType.kLeftRumble, 0);// rumble needs to be explicitly set to 0
          shootController.setRumble(RumbleType.kLeftRumble, 0);
        }
      }
      else {// no target, reset dashboard to let drivers know and disable steering
        resetDash();
        forward = 0;
        steer = 0;
      }
    }
    else {
      resetDash();
    }

    if (ready) {
      // if shoot button is pressed, shoot()
    } // else if manual override buttons are held, shoot()

    if (shootController.getLeftBumperPressed()) {
      phSolenoid2.set(true);
      intakeMotor.set(1.0);
    } else {
      phSolenoid2.set(false);
      intakeMotor.set(-1.0);
      Timer.delay(1.0);
      intakeMotor.set(0);
    }

    SmartDashboard.putNumber("forward", forward);
    myRobot.arcadeDrive(steer, forward);
  }

  public void resetDash() {
    SmartDashboard.putBoolean("has target", false);
    SmartDashboard.putBoolean("distance good", false);
    SmartDashboard.putBoolean("angle good", false);
    SmartDashboard.putBoolean("ready", false);
    driverController.setRumble(RumbleType.kLeftRumble, 0);
    shootController.setRumble(RumbleType.kLeftRumble, 0);
  }

  public void setLed(int r, int g, int b) {

  }

  public void handleDash() {

    p_dist = SmartDashboard.getNumber("kP_dist", 0);
    d_dist = SmartDashboard.getNumber("kD_dist", 0);
    p_angle = SmartDashboard.getNumber("kP_angle", 0);
    d_angle = SmartDashboard.getNumber("kD_angle", 0);

    if (p_dist != kP_dist) {
      kP_dist = p_dist;
      SmartDashboard.putNumber("kP_dist", kP_dist);
      PID_dist.setP(kP_dist);
    }
    if (d_dist != kD_dist) {
      kD_dist = d_dist;
      SmartDashboard.putNumber("kD_dist", kP_dist);
      PID_dist.setD(kD_dist);
    }
    if (p_angle != kP_angle) {
      kP_angle = p_angle;
      SmartDashboard.putNumber("kP_angle", kP_dist);
      PID_angle.setP(kP_angle);
    }
    if (d_angle != kD_angle) {
      kD_angle = d_angle;
      SmartDashboard.putNumber("kD_angle", kP_dist);
      PID_angle.setD(kD_angle);
    }
  }
}

/**
  * SmartDashboard.putNumber("target ID", target.getFiducialId());
  * SmartDashboard.putNumber("X (forward)",
  * target.getBestCameraToTarget().getTranslation().getX());
  * SmartDashboard.putNumber("Y (left)",
  * target.getBestCameraToTarget().getTranslation().getY());
  * SmartDashboard.putNumber("Z (up)",
  * target.getBestCameraToTarget().getTranslation().getZ());
  * SmartDashboard.putNumber("X ROTATION (roll)",
  * Units.radiansToDegrees(target.getBestCameraToTarget().getRotation().getX()));
  * SmartDashboard.putNumber("Y ROTATION (pitch)",
  * Units.radiansToDegrees(target.getBestCameraToTarget().getRotation().getY()));
  * SmartDashboard.putNumber("Z ROTATION (yaw)",
  * Units.radiansToDegrees(target.getBestCameraToTarget().getRotation().getZ()));
  */