package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Robot extends TimedRobot {
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  private XboxController driverController = new XboxController (0);
  private XboxController shootController = new XboxController (1);
  private DifferentialDrive myRobot;

  private static final int leftLeadID = 1;
  private static final int leftFollow1ID = 2;
  private static final int leftFollow2ID = 3;
  private static final int rightLeadID = 5;
  private static final int rightFollow1ID = 6;
  private static final int rightFollow2ID = 7;
  private static final int intakeID = 10;

  private CANSparkMax leftLead;
  private CANSparkMax leftFollow1;
  private CANSparkMax leftFollow2;
  private CANSparkMax rightLead;
  private CANSparkMax rightFollow1;
  private CANSparkMax rightFollow2;
  private CANSparkMax intakeMotor;
  private final Timer timer = new Timer();

  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  Solenoid phSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  Solenoid phSolenoid2 = new Solenoid(PneumaticsModuleType.REVPH, 1);

  double kP_dist = 0.1;
  double kD_dist = 0.0;
  PIDController PID_dist = new PIDController(kP_dist, 0, kD_dist);
  double kP_angle = 0.1;
  double kD_angle = 0.0;
  PIDController PID_angle = new PIDController(kP_dist, 0, kD_dist);

  double degrees = Units.radiansToDegrees(0);
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);
  DigitalInput back = new DigitalInput(0);
  DigitalInput shoot = new DigitalInput(1);
  DigitalInput intake = new DigitalInput(2);

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("kP_dist", kP_dist);
    SmartDashboard.putNumber("kD_dist", kD_dist);
    SmartDashboard.putNumber("kP_angle", kP_angle);
    SmartDashboard.putNumber("kD_angle", kD_angle);

    camera.setDriverMode(false);
    camera.setPipelineIndex(0);

    leftLead = new CANSparkMax(leftLeadID, MotorType.kBrushless);
    leftFollow1 = new CANSparkMax(leftFollow1ID, MotorType.kBrushless);
    rightLead = new CANSparkMax(rightLeadID, MotorType.kBrushless);
    rightFollow1 = new CANSparkMax(rightFollow1ID, MotorType.kBrushless);
    leftFollow2 = new CANSparkMax(leftFollow2ID, MotorType.kBrushless);
    rightFollow2 = new CANSparkMax(rightFollow2ID, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);

    leftLead.restoreFactoryDefaults();
    leftFollow1.restoreFactoryDefaults();
    leftFollow2.restoreFactoryDefaults();
    rightLead.restoreFactoryDefaults();
    rightFollow1.restoreFactoryDefaults();
    rightFollow2.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();

    int driveCurrentLimit = 60;
    int intakeCurrentLimit = 30;
    leftLead.setSmartCurrentLimit(driveCurrentLimit);
    leftFollow1.setSmartCurrentLimit(driveCurrentLimit);
    leftFollow2.setSmartCurrentLimit(driveCurrentLimit);
    rightLead.setSmartCurrentLimit(driveCurrentLimit);
    rightFollow1.setSmartCurrentLimit(driveCurrentLimit);
    rightFollow2.setSmartCurrentLimit(driveCurrentLimit);
    intakeMotor.setSmartCurrentLimit(intakeCurrentLimit);

    leftFollow1.follow(leftLead);
    leftFollow2.follow(leftLead);
    rightFollow1.follow(rightLead);
    rightFollow2.follow(rightFollow1);

    leftLead.burnFlash();
    leftFollow1.burnFlash();
    leftFollow2.burnFlash();
    rightLead.burnFlash();
    rightFollow1.burnFlash();
    rightFollow2.burnFlash();
    intakeMotor.burnFlash();

    myRobot = new DifferentialDrive(leftLead, rightLead);
    phCompressor.enableDigital();
 }

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }
    
  @Override
  public void autonomousPeriodic() {
    double speed = 0.75;
   if (back.get()) {
    if (timer.get() < 2.0) {
      leftLead.set (speed);
      rightLead.set(speed * -1);
    }} else if (shoot.get()) {
      if (timer.get() < 5.0) {
        phSolenoid.set(true);
        Timer.delay(.50);
        phSolenoid.set(false);
        leftLead.set(speed * -1);
        rightLead.set(speed);
      }
    } else if (intake.get()){
      if (timer.get() < 4.0) {
        phSolenoid.set(true);
        Timer.delay(.50);
        phSolenoid.set(false);
        leftLead.set(speed * -1);
        rightLead.set(speed);
        //intake code added here :)
      }
    }
    else {
        myRobot.stopMotor();
      } 
  }

  double forward = 0;
  double steer = 0;
  double p_dist = 0;
  double d_dist = 0;
  double p_angle = 0;
  double d_angle = 0;

  @Override
  public void teleopPeriodic() {
    p_dist = SmartDashboard.getNumber("kP_dist", 0);
    d_dist = SmartDashboard.getNumber("kD_dist", 0);
    p_angle = SmartDashboard.getNumber("kP_angle", 0);
    d_angle = SmartDashboard.getNumber("kD_angle", 0);
    
    if(p_dist != kP_dist) { 
      kP_dist = p_dist;
      SmartDashboard.putNumber("kP_dist", kP_dist);
      PID_dist.setP(kP_dist);
    }
    if(d_dist != kD_dist) { 
      kD_dist = d_dist;
      SmartDashboard.putNumber("kD_dist", kP_dist);
      PID_dist.setD(kD_dist);
    }
    if(p_angle != kP_angle) { 
      kP_angle = p_angle;
      SmartDashboard.putNumber("kP_angle", kP_dist);
      PID_angle.setP(kP_angle);
    }
    if(d_angle != kD_angle) { 
      kD_angle = d_angle;
      SmartDashboard.putNumber("kD_angle", kP_dist);
      PID_angle.setD(kD_angle);
    }

    double leftYstick = (driverController.getLeftY());
    double rightXstick = (driverController.getRightX());
    forward = leftYstick;
    steer = rightXstick;

    var result = camera.getLatestResult();
   if (driverController.getBButtonPressed()) {
      phSolenoid.set(true);
      Timer.delay(1.0);
    } else if (driverController.getBButtonReleased()) {
      phSolenoid.set(false);
    } 

    if (driverController.getXButtonPressed()){
      phCompressor.disable();
    }

    if(driverController.getAButton()) {
      if(result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = targets.get(0);
        /*SmartDashboard.putNumber("target ID", target.getFiducialId());
        SmartDashboard.putNumber("X (forward)", target.getBestCameraToTarget().getTranslation().getX());
        SmartDashboard.putNumber("Y (left)", target.getBestCameraToTarget().getTranslation().getY());
        SmartDashboard.putNumber("Z (up)", target.getBestCameraToTarget().getTranslation().getZ());
        SmartDashboard.putNumber("X ROTATION (roll)", Units.radiansToDegrees(target.getBestCameraToTarget().getRotation().getX()));
        SmartDashboard.putNumber("Y ROTATION (pitch)", Units.radiansToDegrees(target.getBestCameraToTarget().getRotation().getY()));
        SmartDashboard.putNumber("Z ROTATION (yaw)", Units.radiansToDegrees(target.getBestCameraToTarget().getRotation().getZ()));*/
        forward = -PID_dist.calculate(target.getBestCameraToTarget().getTranslation().getX(), 30);
        steer = PID_angle.calculate(target.getBestCameraToTarget().getTranslation().getY(), 0);
      } else {
        forward = 0;
      }
    }

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
}