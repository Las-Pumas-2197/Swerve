// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.swervemain;

/**Class used to offboard data posting to Shuffleboard or the RIO log.*/
public class telemetry extends SubsystemBase {

  //injected subsystems whose data is to be posted
  private final swervemain subsys_drive;

  /**Telemetry class used to post data to Shuffleboard or the RIO log. Additional subsystems may be added as necessary.
   * @param drive Swerve drive subsystem.
   */
  public telemetry(swervemain drive) {

    //injected subsystems, must be injected to prevent multiple instances of same object
    subsys_drive = drive;
  }

  /**Posts swervemain telemetry data to Shufflboard.*/
  public void swervemain() {
    SmartDashboard.putNumber("Xspeed_des", subsys_drive.getMainTelemetry()[0]);
    SmartDashboard.putNumber("Yspeed_des", subsys_drive.getMainTelemetry()[1]);
    SmartDashboard.putNumber("Zinput", subsys_drive.getMainTelemetry()[2]);
    SmartDashboard.putNumber("Zrot", subsys_drive.getMainTelemetry()[3]);
    SmartDashboard.putNumber("heading_pidout", subsys_drive.getMainTelemetry()[4]);
    SmartDashboard.putNumber("heaidng_ffout", subsys_drive.getMainTelemetry()[5]);
  }

  /**Posts swervemodule telemetry data to shuffleboard. This is a significant amount of data so some formatting 
   * may be needed in Shuffleboard.*/
  public void swervemodule() {

    //FL
    SmartDashboard.putNumber("FL_azimuth_posdes", subsys_drive.getModuleTelemetry()[0][0]);
    SmartDashboard.putNumber("FL_azimuth_posact", subsys_drive.getModuleTelemetry()[0][1]);
    SmartDashboard.putNumber("FL_azimuth_err", subsys_drive.getModuleTelemetry()[0][2]);
    SmartDashboard.putNumber("FL_azimuth_pidout", subsys_drive.getModuleTelemetry()[0][3]);
    SmartDashboard.putNumber("FL_azimuth_ffout", subsys_drive.getModuleTelemetry()[0][4]);
    SmartDashboard.putNumber("FL_drive_veldes", subsys_drive.getModuleTelemetry()[0][5]);
    SmartDashboard.putNumber("FL_drive_velact", subsys_drive.getModuleTelemetry()[0][6]);
    SmartDashboard.putNumber("FL_drive_err", subsys_drive.getModuleTelemetry()[0][7]);
    SmartDashboard.putNumber("FL_drive_ffout", subsys_drive.getModuleTelemetry()[0][8]);

    //FR
    SmartDashboard.putNumber("FL_azimuth_posdes", subsys_drive.getModuleTelemetry()[1][0]);
    SmartDashboard.putNumber("FL_azimuth_posact", subsys_drive.getModuleTelemetry()[1][1]);
    SmartDashboard.putNumber("FL_azimuth_err", subsys_drive.getModuleTelemetry()[1][2]);
    SmartDashboard.putNumber("FL_azimuth_pidout", subsys_drive.getModuleTelemetry()[1][3]);
    SmartDashboard.putNumber("FL_azimuth_ffout", subsys_drive.getModuleTelemetry()[1][4]);
    SmartDashboard.putNumber("FL_drive_veldes", subsys_drive.getModuleTelemetry()[1][5]);
    SmartDashboard.putNumber("FL_drive_velact", subsys_drive.getModuleTelemetry()[1][6]);
    SmartDashboard.putNumber("FL_drive_err", subsys_drive.getModuleTelemetry()[1][7]);
    SmartDashboard.putNumber("FL_drive_ffout", subsys_drive.getModuleTelemetry()[1][8]);

    //RL
    SmartDashboard.putNumber("FL_azimuth_posdes", subsys_drive.getModuleTelemetry()[2][0]);
    SmartDashboard.putNumber("FL_azimuth_posact", subsys_drive.getModuleTelemetry()[2][1]);
    SmartDashboard.putNumber("FL_azimuth_err", subsys_drive.getModuleTelemetry()[2][2]);
    SmartDashboard.putNumber("FL_azimuth_pidout", subsys_drive.getModuleTelemetry()[2][3]);
    SmartDashboard.putNumber("FL_azimuth_ffout", subsys_drive.getModuleTelemetry()[2][4]);
    SmartDashboard.putNumber("FL_drive_veldes", subsys_drive.getModuleTelemetry()[2][5]);
    SmartDashboard.putNumber("FL_drive_velact", subsys_drive.getModuleTelemetry()[2][6]);
    SmartDashboard.putNumber("FL_drive_err", subsys_drive.getModuleTelemetry()[2][7]);
    SmartDashboard.putNumber("FL_drive_ffout", subsys_drive.getModuleTelemetry()[2][8]);

    //RR
    SmartDashboard.putNumber("FL_azimuth_posdes", subsys_drive.getModuleTelemetry()[3][0]);
    SmartDashboard.putNumber("FL_azimuth_posact", subsys_drive.getModuleTelemetry()[3][1]);
    SmartDashboard.putNumber("FL_azimuth_err", subsys_drive.getModuleTelemetry()[3][2]);
    SmartDashboard.putNumber("FL_azimuth_pidout", subsys_drive.getModuleTelemetry()[3][3]);
    SmartDashboard.putNumber("FL_azimuth_ffout", subsys_drive.getModuleTelemetry()[3][4]);
    SmartDashboard.putNumber("FL_drive_veldes", subsys_drive.getModuleTelemetry()[3][5]);
    SmartDashboard.putNumber("FL_drive_velact", subsys_drive.getModuleTelemetry()[3][6]);
    SmartDashboard.putNumber("FL_drive_err", subsys_drive.getModuleTelemetry()[3][7]);
    SmartDashboard.putNumber("FL_drive_ffout", subsys_drive.getModuleTelemetry()[3][8]);
  }

  @Override
  public void periodic() {}
}
