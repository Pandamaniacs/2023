// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.Timer;
import java.lang.annotation.Target;
import java.util.List;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * This is a demo program showing the use of OpenCV to do vision processing. The image is acquired
 * from the USB camera, then a rectangle is put on the image and sent to the dashboard. OpenCV has
 * many methods for different types of processing.
 */
public class Robot extends TimedRobot {
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  private XboxController driverController = new XboxController (0);
  private XboxController shootController = new XboxController (1);
  private DifferentialDrive myRobot;

  private static final int leftDeviceID = 1;
  private static final int leftFollowID = 2;
 // private static final int leftFollow2ID = 3;
  private static final int rightDeviceID = 3;
  private static final int rightFollowID = 4;
 // private static final int rightFollow2ID = 6;
 private static final int intakesID = 10;

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;  
  private CANSparkMax leftFollow;
  private CANSparkMax rightFollow;
  //private CANSparkMax leftFollow2;
  //private CANSparkMax rightFollow2;
  private CANSparkMax intakes;
  private final Timer timer = new Timer();

  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  Solenoid phSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  Solenoid phSolenoid2 = new Solenoid(PneumaticsModuleType.REVPH, 1);

 // private XboxController xbox = new XboxController (0);
  final double P_GAIN = 0.1;
  final double D_GAIN = 0.0;
  PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);
  //angle between the horizontal and the camera
  double degrees = Units.radiansToDegrees(0);
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);
  DigitalInput back = new DigitalInput(0);
  DigitalInput shoot = new DigitalInput(1);
  DigitalInput intake = new DigitalInput(2);

  @Override
  public void robotInit() {
    camera.setDriverMode(false);
    camera.setPipelineIndex(0);

    leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
    leftFollow = new CANSparkMax(leftFollowID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);
    rightFollow = new CANSparkMax(rightFollowID, MotorType.kBrushless);
    //leftFollow2 = new CANSparkMax(leftFollow2ID, MotorType.kBrushless);
    //rightFollow2 = new CANSparkMax(rightFollow2ID, MotorType.kBrushless);
    intakes = new CANSparkMax(intakesID, MotorType.kBrushless);
    leftMotor.restoreFactoryDefaults();
    leftFollow.restoreFactoryDefaults();
    //leftFollow2.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    rightFollow.restoreFactoryDefaults();
    //rightFollow2.restoreFactoryDefaults();
    intakes.restoreFactoryDefaults();
    //int dtCurrentLimit = 60;
    //int intakeCurrentLimit = 30;
   /* leftMotor.setSmartCurrentLimit(dtCurrentLimit);
    leftFollow.setSmartCurrentLimit(dtCurrentLimit);
    leftFollow2.setSmartCurrentLimit(dtCurrentLimit);
    rightMotor.setSmartCurrentLimit(dtCurrentLimit);
    rightFollow.setSmartCurrentLimit(dtCurrentLimit);
    rightFollow2.setSmartCurrentLimit(dtCurrentLimit);
    intakes.setSmartCurrentLimit(intakeCurrentLimit); */
    leftFollow.follow(leftMotor);
    //leftFollow2.follow(leftMotor);
    rightFollow.follow(rightMotor);
   // rightFollow2.follow(rightFollow);

    leftMotor.burnFlash();
    leftFollow.burnFlash();
    rightMotor.burnFlash();
    rightFollow.burnFlash();
    intakes.burnFlash();

    myRobot = new DifferentialDrive(leftMotor, rightMotor);
    phCompressor.enableDigital();
 }

  PIDController dController = new PIDController(0.1, 0.0, 0.0);
  final double goal = 30;
  //CANSparkMax left = new CANSparkMax(1, MotorType.kBrushless);
  //CANSparkMax right = new CANSparkMax(3, MotorType.kBrushless);

    @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    }
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double speed = 0.75;
   if (back.get()) {
    if (timer.get() < 2.0) {
      leftMotor.set (speed);
      rightMotor.set(speed * -1);
    }} else if (shoot.get()) {
      if (timer.get() < 5.0) {
        phSolenoid.set(true);
        Timer.delay(.50);
        phSolenoid.set(false);
        leftMotor.set(speed * -1);
        rightMotor.set(speed);
      }
    } else if (intake.get()){
      if (timer.get() < 4.0) {
        phSolenoid.set(true);
        Timer.delay(.50);
        phSolenoid.set(false);
        leftMotor.set(speed * -1);
        rightMotor.set(speed);
        //intake code added here :)
      }
    }
    else {
        myRobot.stopMotor();
      } 
  }   
  @Override
  public void teleopPeriodic() { 
    double leftYstick = (driverController.getLeftY());
    double rightXstick = (driverController.getRightX());
    //phSolenoid.set(true);
    double forward;

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

    if(driverController.getAButton() || true) {

      if(result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = targets.get(0);
        //degrees = Units.radiansToDegrees(result.getBestTarget().getYaw());
        //System.out.println("degrees: " + degrees);
        SmartDashboard.putNumber("target ID", target.getFiducialId());
        SmartDashboard.putNumber("X (forward)", target.getBestCameraToTarget().getTranslation().getX());
        SmartDashboard.putNumber("Y (left)", target.getBestCameraToTarget().getTranslation().getY());
        SmartDashboard.putNumber("Z (up)", target.getBestCameraToTarget().getTranslation().getZ());
        SmartDashboard.putNumber("X ROTATION (roll)", Units.radiansToDegrees(target.getBestCameraToTarget().getRotation().getX()));
        SmartDashboard.putNumber("Y ROTATION (pitch)", Units.radiansToDegrees(target.getBestCameraToTarget().getRotation().getY()));
        SmartDashboard.putNumber("Z ROTATION (yaw)", Units.radiansToDegrees(target.getBestCameraToTarget().getRotation().getZ()));
        //SmartDashboard.putNumber("degrees", degrees);
        //forward = -controller.calculate(degrees, GOAL_RANGE_METERS);
        forward = -controller.calculate(result.getBestTarget().getPitch(), 0);
      } else {
        forward = 0;
      }
    }

    if (shootController.getLeftBumperPressed()) {
      phSolenoid2.set(true);
      intakes.set(1.0);
    } else {
      phSolenoid2.set(false);
      intakes.set(-1.0);
      Timer.delay(1.0);
      intakes.set(0);
    }

    SmartDashboard.putNumber("forward", forward);
    SmartDashboard.putBoolean("camera has target", result.hasTargets());
    SmartDashboard.putNumber("latency", result.getLatencyMillis());
    double current = phCompressor.getCurrent();
    double pressure = phCompressor.getPressure();
    SmartDashboard.getNumber("pressure", pressure);
    SmartDashboard.putNumber("current", current);
    myRobot.arcadeDrive(rightXstick, leftYstick);
  }
}