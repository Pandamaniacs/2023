package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

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
  ADIS16470_IMU gyro = new ADIS16470_IMU();

  private double kP = .05;
  private double kI = 0;
  private double kD = 0;
  private double kP_bal = .035;
  private double kI_bal = 0;
  private double kD_bal = .005;
  PIDController PID = new PIDController(kP, kI, kD);
  PIDController PID_bal = new PIDController(kP_bal, kI_bal, kD_bal);
  double set_point;

  @Override
  public void robotInit() {

    initMotors();
    
    myRobot = new DifferentialDrive(leftLeader, rightLeader);
    SmartDashboard.putNumber("gyro", 0);
    SmartDashboard.putNumber("kP bal", 0);
    SmartDashboard.putNumber("kD bal", 0);
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

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    gyro.reset();
    set_point = gyro.getAngle();
    SmartDashboard.putNumber("x_axis", gyro.getXComplementaryAngle());
    SmartDashboard.putNumber("y axis", gyro.getYComplementaryAngle());
  }

  private final double deposit = 1;
  private final double exitCommunity = deposit + 3.5;
  private final double re_enter = exitCommunity + 0.5;
  private final double speed = 0.4;
  double error = -gyro.getRate();

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("y axis", gyro.getYComplementaryAngle());
    // // if it's between start time and one second, intake in the cube/poop out the cube
    // if(timer.get() > 0 && timer.get() < deposit) {
    //   intakeMotor.set(-1);
    // // if the time is between one second and 3.5 seconds, intake stops and robot moves out of community
    // } else if(timer.get() > deposit && timer.get() < exitCommunity) {
    //   intakeMotor.set(0);
    //   for (double a = gyro.getYComplementaryAngle(); a < 20; a++) {
    //  //ensures that the robot motors autocorrect itself through the gyro
    //     myRobot.arcadeDrive(0, speed - kP*error);
    //   }
    //   // time is between 3.5 seconds and 4.25
    // } else if(timer.get() > exitCommunity && timer.get() < re_enter) {
    //   intakeMotor.set(0);
    //   myRobot.arcadeDrive(0, -speed);
    // } else if (timer.get() > re_enter && timer.get() < 15) {
    //   for (double a = gyro.getYComplementaryAngle(); a > 1; a++) {
    //     //ensures that the robot motors autocorrect itself through the gyro
    //   myRobot.arcadeDrive(0, .05 + kP*error);
    // } }
    // else {
    //   myRobot.stopMotor();
    //   intakeMotor.set(0);
    // }
    myRobot.arcadeDrive(0, PID_bal.calculate(gyro.getYComplementaryAngle(), 0));
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
    } else {
      intakeMotor.set(0);
    }
    SmartDashboard.putNumber("y axis", gyro.getYComplementaryAngle());
    myRobot.arcadeDrive(steer, forward);
  }
}