package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.controller.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
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

  private final double kP = .05;
  private final double kI = 0;
  private final double kD = 0;
  PIDController PID = new PIDController(kP, kI, kD);

  AHRS ahrs;

  private final Timer timer = new Timer();

  @Override
  public void robotInit() {

    initMotors();
    myRobot = new DifferentialDrive(leftLeader, rightLeader);
    ahrs = new AHRS(SPI.Port.kMXP); 
  }

  int first = 0;
  
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

    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollow1.setIdleMode(IdleMode.kBrake);
    leftFollow2.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollow1.setIdleMode(IdleMode.kBrake);
    rightFollow2.setIdleMode(IdleMode.kBrake);
    intakeMotor.setIdleMode(IdleMode.kBrake);

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

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }

  private final double deposit = 1;
  private final double exitCommunity = deposit + 2.5;
  private final double charging = exitCommunity + 1.85;
  private final double speed = .5;
  double pitchAngle;
  @Override
  public void autonomousPeriodic() {
    pitchAngle = ahrs.getPitch();
    // if it's between start time and one second, intake in the cube/poop out the cube
    if(timer.get() > 0 && timer.get() < deposit) {
      intakeMotor.set(-1);
    // if the time is between one second and 3.5 seconds, intake stops and robot moves out of community
    } else if(timer.get() > deposit && timer.get() < exitCommunity) {
      intakeMotor.set(0);
      myRobot.arcadeDrive(0, speed);
      // time is between 3.5 seconds and 4.25
    } else if(timer.get() > exitCommunity && timer.get() < charging) {
      intakeMotor.set(0);
      myRobot.arcadeDrive(0, -speed);
      // myRobot.arcadeDrive(0, PID.calculate(pitchAngle, 0));
    } else {
      myRobot.stopMotor();
      intakeMotor.set(0);
    }
  }
  private final double turnMultiplier = .6;
  private final double speedMultiplier = 1;
  @Override
  public void teleopPeriodic() {
    double forward;
    double steer;
    if(driverController.getLeftBumper() && driverController.getRightBumper()) {
      forward = (driverController.getLeftY()*speedMultiplier);
      steer = (driverController.getRightX()*turnMultiplier);
    } else if(driverController.getLeftBumper() || driverController.getRightBumper()) {
      forward = (driverController.getLeftY()*speedMultiplier/2);
      steer = (driverController.getRightX()*turnMultiplier/2);
    } else {
      forward = (driverController.getLeftY()*speedMultiplier/3);
      steer = (driverController.getRightX()*turnMultiplier/2);
    }

    // if(driverController.getLeftTriggerAxis() >= .1 || driverController.getRightTriggerAxis() >= .1) {
    //   forward = -forward;
    //   steer = -steer;
    // }

    intakeMotor.set(0);
    if(shootController.getLeftBumper()) {
      intakeMotor.set(.5);
    } else if(shootController.getLeftTriggerAxis() > .05){
      intakeMotor.set(shootController.getLeftTriggerAxis());
    } else if(shootController.getRightBumper()){
      intakeMotor.set(-.5);
    } else if(shootController.getRightTriggerAxis() > .05){
      intakeMotor.set(-shootController.getRightTriggerAxis());
    }

    myRobot.arcadeDrive(steer, forward);
  }
}