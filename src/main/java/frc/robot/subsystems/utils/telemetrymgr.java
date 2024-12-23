// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.swervemain;

/**Class used to offboard data posting to Shuffleboard or the RIO log.*/
public class telemetrymgr extends SubsystemBase {

  //injected subsystems whose data is to be posted
  private final swervemain subsys_drive;

  /**Telemetry class used to post data to Shuffleboard or the RIO log. Additional subsystems may be added as necessary.
   * @param drive Swerve drive subsystem.
   */
  public telemetrymgr(swervemain drive) {

    //injected subsystems, must be injected to prevent multiple instances of same object
    subsys_drive = drive;
  }

  /**Posts swervemain telemetry data to Shufflboard.*/
  public void swervemain() {
    SmartDashboard.putNumber("Xspeed_des", subsys_drive.getMainTelemetry()[0]);
    SmartDashboard.putNumber("Yspeed_des", subsys_drive.getMainTelemetry()[1]);
    SmartDashboard.putNumber("Zinput", subsys_drive.getMainTelemetry()[2]);
    SmartDashboard.putNumber("Zrot", subsys_drive.getMainTelemetry()[3]);
    SmartDashboard.putNumber("heading_act", subsys_drive.getMainTelemetry()[4]);
    SmartDashboard.putNumber("heading_des", subsys_drive.getMainTelemetry()[5]);
    SmartDashboard.putNumber("heading_pidout", subsys_drive.getMainTelemetry()[6]);
    SmartDashboard.putNumber("heaidng_ffout", subsys_drive.getMainTelemetry()[7]);
  }

  public void currentpose(){
    SmartDashboard.putNumber("currentpose_X", subsys_drive.getPose().getX());
    SmartDashboard.putNumber("currentpose_Y", subsys_drive.getPose().getY());
    SmartDashboard.putNumber("currentpose_heading", subsys_drive.getPose().getRotation().getRadians());
  }

  public void swervemoduleFL() {
    SmartDashboard.putNumber("FL_azimuth_posdes", subsys_drive.getModuleTelemetry()[0][0]);
    SmartDashboard.putNumber("FL_azimuth_posact", subsys_drive.getModuleTelemetry()[0][1]);
    SmartDashboard.putNumber("FL_azimuth_err", subsys_drive.getModuleTelemetry()[0][2]);
    SmartDashboard.putNumber("FL_azimuth_pidout", subsys_drive.getModuleTelemetry()[0][3]);
    SmartDashboard.putNumber("FL_azimuth_ffout", subsys_drive.getModuleTelemetry()[0][4]);
    SmartDashboard.putNumber("FL_drive_veldes", subsys_drive.getModuleTelemetry()[0][5]);
    SmartDashboard.putNumber("FL_drive_velact", subsys_drive.getModuleTelemetry()[0][6]);
    SmartDashboard.putNumber("FL_drive_err", subsys_drive.getModuleTelemetry()[0][7]);
    SmartDashboard.putNumber("FL_drive_ffout", subsys_drive.getModuleTelemetry()[0][8]);
  }

  public void swervemoduleFR() {
    SmartDashboard.putNumber("FR_azimuth_posdes", subsys_drive.getModuleTelemetry()[1][0]);
    SmartDashboard.putNumber("FR_azimuth_posact", subsys_drive.getModuleTelemetry()[1][1]);
    SmartDashboard.putNumber("FR_azimuth_err", subsys_drive.getModuleTelemetry()[1][2]);
    SmartDashboard.putNumber("FR_azimuth_pidout", subsys_drive.getModuleTelemetry()[1][3]);
    SmartDashboard.putNumber("FR_azimuth_ffout", subsys_drive.getModuleTelemetry()[1][4]);
    SmartDashboard.putNumber("FR_drive_veldes", subsys_drive.getModuleTelemetry()[1][5]);
    SmartDashboard.putNumber("FR_drive_velact", subsys_drive.getModuleTelemetry()[1][6]);
    SmartDashboard.putNumber("FR_drive_err", subsys_drive.getModuleTelemetry()[1][7]);
    SmartDashboard.putNumber("FR_drive_ffout", subsys_drive.getModuleTelemetry()[1][8]);
  }

  public void swervemoduleRL() {
    SmartDashboard.putNumber("RL_azimuth_posdes", subsys_drive.getModuleTelemetry()[2][0]);
    SmartDashboard.putNumber("RL_azimuth_posact", subsys_drive.getModuleTelemetry()[2][1]);
    SmartDashboard.putNumber("RL_azimuth_err", subsys_drive.getModuleTelemetry()[2][2]);
    SmartDashboard.putNumber("RL_azimuth_pidout", subsys_drive.getModuleTelemetry()[2][3]);
    SmartDashboard.putNumber("RL_azimuth_ffout", subsys_drive.getModuleTelemetry()[2][4]);
    SmartDashboard.putNumber("RL_drive_veldes", subsys_drive.getModuleTelemetry()[2][5]);
    SmartDashboard.putNumber("RL_drive_velact", subsys_drive.getModuleTelemetry()[2][6]);
    SmartDashboard.putNumber("RL_drive_err", subsys_drive.getModuleTelemetry()[2][7]);
    SmartDashboard.putNumber("RL_drive_ffout", subsys_drive.getModuleTelemetry()[2][8]);
  }

  public void swervemoduleRR() {
    SmartDashboard.putNumber("RR_azimuth_posdes", subsys_drive.getModuleTelemetry()[3][0]);
    SmartDashboard.putNumber("RR_azimuth_posact", subsys_drive.getModuleTelemetry()[3][1]);
    SmartDashboard.putNumber("RR_azimuth_err", subsys_drive.getModuleTelemetry()[3][2]);
    SmartDashboard.putNumber("RR_azimuth_pidout", subsys_drive.getModuleTelemetry()[3][3]);
    SmartDashboard.putNumber("RR_azimuth_ffout", subsys_drive.getModuleTelemetry()[3][4]);
    SmartDashboard.putNumber("RR_drive_veldes", subsys_drive.getModuleTelemetry()[3][5]);
    SmartDashboard.putNumber("RR_drive_velact", subsys_drive.getModuleTelemetry()[3][6]);
    SmartDashboard.putNumber("RR_drive_err", subsys_drive.getModuleTelemetry()[3][7]);
    SmartDashboard.putNumber("RR_drive_ffout", subsys_drive.getModuleTelemetry()[3][8]);
  }

  @Override
  public void periodic() {}
}
