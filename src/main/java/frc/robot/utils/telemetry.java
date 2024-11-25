// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.driverinputs.HIDs;
import frc.robot.drivetrain.swervedrive;

public class telemetry extends SubsystemBase {

  //instantiate objects for getting telemetry
  private final swervedrive drivetrain;
  private final HIDs hids;

  /**
   * Telemetry subsystem for diagnostics on Shuffleboard.
   */
  public telemetry() {
    //declare objects
    drivetrain = new swervedrive();
    hids = new HIDs();
  }

  public void runHIDTelemetry() {

    //write joystick data to dashboard
    double [] velocityData = hids.getXboxVelocityData();

    //write joystick data to dashboard
    SmartDashboard.putNumber("Xspeeddes", velocityData[0]);
    SmartDashboard.putNumber("Yspeeddes", velocityData[1]);
    SmartDashboard.putNumber("Zrotdes", velocityData[2]);
  }

  public void runSwerveTelemetry() {
    //get drivetrain data
    double[] drivetrainData = drivetrain.GetDrivetrainData();

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
    double[] drivetraindesiredData = drivetrain.GetDesiredData();

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
    double[] drivePIDoutputData = drivetrain.GetPIDoutData();

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
  }

  @Override
  public void periodic() {
    
  }
}
