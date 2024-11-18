// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.driverinputs.HIDs;
import frc.robot.drivetrain.swervedrive;
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //construct swervedrive object and HID object
  private final swervedrive swervedrive = new swervedrive();
  private final HIDs HIDs = new HIDs();

  //construct velocity data array for speeds
  private double[] velocityData;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //get drivetrain data
    double[] drivetrainData = swervedrive.GetDrivetrainData();

    //write array to dashboard
    SmartDashboard.putNumber("FLDriveVelActual", drivetrainData[0]);
    SmartDashboard.putNumber("FLTurnPosActual", drivetrainData[1]);
    SmartDashboard.putNumber("FRDriveVelActual", drivetrainData[2]);
    SmartDashboard.putNumber("FRTurnPosActual", drivetrainData[3]);
    SmartDashboard.putNumber("RLDriveVelActual", drivetrainData[4]);
    SmartDashboard.putNumber("RLTurnPosActual", drivetrainData[5]);
    SmartDashboard.putNumber("RRDriveVelActual", drivetrainData[6]);
    SmartDashboard.putNumber("RRTurnPosActual", drivetrainData[7]);
    SmartDashboard.putNumber("HeadingActual", drivetrainData[8]);

    //get desired drivetrain data
    double[] drivetraindesiredData = swervedrive.GetDesiredData();

    //write desired data array to dashboard
    SmartDashboard.putNumber("FLDriveVelDesired", drivetraindesiredData[0]);
    SmartDashboard.putNumber("FLTurnPosDesired", drivetraindesiredData[1]);
    SmartDashboard.putNumber("FRDriveVelDesired", drivetraindesiredData[2]);
    SmartDashboard.putNumber("FRTurnPosDesired", drivetraindesiredData[3]);
    SmartDashboard.putNumber("RLDriveVelDesired", drivetraindesiredData[4]);
    SmartDashboard.putNumber("RLTurnPosDesired", drivetraindesiredData[5]);
    SmartDashboard.putNumber("RRDriveVelDesired", drivetraindesiredData[6]);
    SmartDashboard.putNumber("RRTurnPosDesired", drivetraindesiredData[7]);
    SmartDashboard.putNumber("HeadingDesired", drivetraindesiredData[8]);

    //get PID output data
    double[] drivePIDoutputData = swervedrive.GetPIDoutData();

    //write PID output data to dashboard
    SmartDashboard.putNumber("FLdrivePIDout", drivePIDoutputData[0]);
    SmartDashboard.putNumber("FRdrivePIDout", drivePIDoutputData[1]);
    SmartDashboard.putNumber("RLdrivePIDout", drivePIDoutputData[2]);
    SmartDashboard.putNumber("RRdrivePIDout", drivePIDoutputData[3]);
    SmartDashboard.putNumber("FLturnPIDout", drivePIDoutputData[4]);
    SmartDashboard.putNumber("FRturnPIDout", drivePIDoutputData[5]);
    SmartDashboard.putNumber("RLturnPIDout", drivePIDoutputData[6]);
    SmartDashboard.putNumber("RRturnPIDout", drivePIDoutputData[7]);
    SmartDashboard.putNumber("HeadingPIDout", drivePIDoutputData[8]);

    //write joystick data to dashboard
    velocityData = HIDs.getXboxVelocityData();

    //write joystick data to dashboard
    SmartDashboard.putNumber("Xspeeddes", velocityData[0]);
    SmartDashboard.putNumber("Yspeeddes", velocityData[1]);
    SmartDashboard.putNumber("Zrotdes", velocityData[2]);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    swervedrive.driveIKinVCL(velocityData[0], velocityData[1], velocityData[2], HIDs.getXboxVelocityData()[3], false);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
