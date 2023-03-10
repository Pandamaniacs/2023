package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

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

  private final Timer timer = new Timer();

  DigitalInput back = new DigitalInput(0);
  DigitalInput shoot = new DigitalInput(1);
  DigitalInput intake = new DigitalInput(2);

  @Override
  public void robotInit() {

    initMotors();

    myRobot = new DifferentialDrive(leftLeader, rightLeader);
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

  private final int seconds = 2;
  private final double speed = .5;
  @Override
  public void autonomousPeriodic() {
    if(timer.get() < seconds) {
      myRobot.arcadeDrive(0, speed);
    } else {
      myRobot.arcadeDrive(0, 0);
    }
  }

  @Override
  public void teleopPeriodic() {
    double forward;
    double steer;
    if(driverController.getLeftBumper() && driverController.getRightBumper()) {
      forward = (driverController.getLeftY()/4);
      steer = (driverController.getRightX()/4);
    } else if(driverController.getLeftBumper() || driverController.getRightBumper()) {
      forward = (driverController.getLeftY()/2);
      steer = (driverController.getRightX()/2);
    } else {
      forward = (driverController.getLeftY());
      steer = (driverController.getRightX());
    }

    if(shootController.getLeftBumper()) {
      intakeMotor.set(1);
    } else if(shootController.getLeftTriggerAxis() > .05){
      intakeMotor.set(shootController.getLeftTriggerAxis());
    } else if(shootController.getRightBumper()){
      intakeMotor.set(-1);
    } else if(shootController.getRightTriggerAxis() > .05){
      intakeMotor.set(-shootController.getRightTriggerAxis());
    }

    myRobot.arcadeDrive(steer, forward);
  }
}