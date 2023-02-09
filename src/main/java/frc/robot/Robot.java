// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.PhotonUtils;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private XboxController driverController = new XboxController (0);
  private XboxController shootController = new XboxController (1);
  private DifferentialDrive myRobot;

  private static final int leftDeviceID = 1;
  private static final int leftFollowID = 2;
  private static final int leftFollow2ID = 3;
  private static final int rightDeviceID = 4;
  private static final int rightFollowID = 5;
  private static final int rightFollow2ID = 6;
  private static final int intake = 9;

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;  
  private CANSparkMax leftFollow;
  private CANSparkMax rightFollow;
  private CANSparkMax leftFollow2;
  private CANSparkMax rightFollow2;
  private CANSparkMax intakes;
  private final Timer timer = new Timer();
  Compressor ph = new Compressor(0, PneumaticsModuleType.REVPH);
  Solenoid sol1Solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
      // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("photonvision");
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    CameraServer.startAutomaticCapture();
    leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
    leftFollow = new CANSparkMax(leftFollowID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);
    rightFollow = new CANSparkMax(rightFollowID, MotorType.kBrushless);
    leftFollow2 = new CANSparkMax(leftFollow2ID, MotorType.kBrushless);
    rightFollow2 = new CANSparkMax(rightFollow2ID, MotorType.kBrushless);
    intakes = new CANSparkMax(intake, MotorType.kBrushless);
    leftMotor.restoreFactoryDefaults();
    leftFollow.restoreFactoryDefaults();
    leftFollow2.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    rightFollow.restoreFactoryDefaults();
    rightFollow2.restoreFactoryDefaults();
    intakes.restoreFactoryDefaults();
    int dtCurrentLimit = 60;
    int intakeCurrentLimit = 30;

    leftMotor.setSmartCurrentLimit(dtCurrentLimit);
    leftFollow.setSmartCurrentLimit(dtCurrentLimit);
    leftFollow2.setSmartCurrentLimit(dtCurrentLimit);
    rightMotor.setSmartCurrentLimit(dtCurrentLimit);
    rightFollow.setSmartCurrentLimit(dtCurrentLimit);
    rightFollow2.setSmartCurrentLimit(dtCurrentLimit);
    intakes.setSmartCurrentLimit(intakeCurrentLimit);
    leftFollow.follow(leftMotor);
    leftFollow2.follow(leftMotor);
    rightFollow.follow(rightMotor);
    rightFollow2.follow(rightFollow);

    leftMotor.burnFlash();
    leftFollow.burnFlash();
    //leftFollow2.burnFlash();
    rightMotor.burnFlash();
    rightFollow.burnFlash();
    rightFollow2.burnFlash();
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();
    // Check if the latest result has any targets.
    // Get the current best target.
    PhotonTrackedTarget target = result.getBestTarget();
    // Get information from target.
    int targetID = target.getFiducialId();
    double poseAmbiguity = target.getPoseAmbiguity();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
    ph.enableAnalog(80, 120);
    myRobot = new DifferentialDrive(rightMotor, leftMotor);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double leftYstick = (driverController.getLeftY());
    double rightXstick = (driverController.getRightX());

    if (driverController.getLeftBumper()) {
      sol1Solenoid.set(true);
    } else {
      sol1Solenoid.set(false);
    }

    //if (shootController.getAButton()) {
      // Vision-alignment mode
      // Query the latest result from PhotonVision
      /*var result = camera.getLatestResult();

       if (result.hasTargets()) {
          // First calculate range
          double range =
                  PhotonUtils.calculateDistanceToTargetMeters(
                          //CAMERA_HEIGHT_METERS,
                          //TARGET_HEIGHT_METERS,
                          //CAMERA_PITCH_RADIANS,
                          Units.degreesToRadians(result.getBestTarget().getPitch()));

          // Use this range as the measurement we give to the PID controller.
          // -1.0 required to ensure positive PID controller effort _increases_ range
          double forwardSpeed = -shootController.calculate(range, //GOAL_RANGE_METERS
          ); 
  } */
  myRobot.arcadeDrive(rightXstick, leftYstick); 
//}
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
