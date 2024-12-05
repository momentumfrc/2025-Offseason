// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.ControlMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private MecanumDrive m_robotDrive;
  private XboxController controller = new XboxController(0);

  private CANVenom frontLeft = new CANVenom(3);
  private CANVenom rearLeft = new CANVenom(4);
  private CANVenom frontRight = new CANVenom(5);
  private CANVenom rearRight = new CANVenom(1);
  private double MAX_SPEED = 1000;
  private double driveKP = 0.2;
  private double driveKFF = 0.4;

  private ShuffleboardTab drivePID = Shuffleboard.getTab("Drive PID");
  private GenericEntry kP = drivePID.add("kP", driveKP).getEntry();
  private GenericEntry kFF = drivePID.add("kFF", driveKFF).getEntry();

  private void setFrontLeftWithPID(double value) {
    double speed = value * MAX_SPEED;
    frontLeft.setCommand(ControlMode.SpeedControl, speed);
    SmartDashboard.putNumber("Front Left Speed Setpoint", speed);
  }

  private void setBackLeftWithPID(double value) {
    double speed = value * MAX_SPEED;
    rearLeft.setCommand(ControlMode.SpeedControl, speed);
    SmartDashboard.putNumber("Back Left Speed Setpoint", speed);
  }

  private void setFrontRightWithPID(double value) {
    double speed = value * MAX_SPEED;
    frontRight.setCommand(ControlMode.SpeedControl, speed);
    SmartDashboard.putNumber("Front Right Speed Setpoint", speed);
  }

  private void setBackRightWithPID(double value) {
    double speed = value * MAX_SPEED;
    rearRight.setCommand(ControlMode.SpeedControl, speed);
    SmartDashboard.putNumber("Back Right Speed Setpoint", speed);
  }

  private void setDriveConstants() {
    double smartdashboardKp = kP.getDouble(driveKP);
    double smartdashboardKFF = kFF.getDouble(driveKFF);
    if (driveKP!=smartdashboardKp){
      driveKP = smartdashboardKp;
      frontLeft.setKP(driveKP);
      frontRight.setKP(driveKP);
      rearLeft.setKP(driveKP);
      rearRight.setKP(driveKP);
    }
    if (driveKFF!=smartdashboardKFF){
      driveKFF = smartdashboardKFF;
      frontLeft.setKF(driveKFF);
      frontRight.setKF(driveKFF);
      rearLeft.setKF(driveKFF);
      rearRight.setKF(driveKFF);
    }
  }

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    frontRight.setInverted(true);
    rearRight.setInverted(true);

    frontLeft.setPID(driveKP,0,0,driveKFF,0);
    frontRight.setPID(driveKP,0,0,driveKFF,0);
    rearLeft.setPID(driveKP,0,0,driveKFF,0);
    rearRight.setPID(driveKP,0,0,driveKFF,0);

    m_robotDrive = new MecanumDrive(this::setFrontLeftWithPID, this::setBackLeftWithPID, this::setFrontRightWithPID, this::setBackRightWithPID);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    setDriveConstants();
  }

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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    setDriveKP();

    m_robotDrive.driveCartesian(-controller.getLeftY(), controller.getLeftX(), controller.getRightX());

    SmartDashboard.putNumber("Front Right Speed", frontRight.getSpeed());
    SmartDashboard.putNumber("Front Left Speed", frontLeft.getSpeed());
    SmartDashboard.putNumber("Back Right Speed", rearRight.getSpeed());
    SmartDashboard.putNumber("Back Left Speed", rearLeft.getSpeed());
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
