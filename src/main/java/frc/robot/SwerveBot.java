// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drivetrain.swervemain;
import frc.robot.subsystems.utils.controllermgr;
import frc.robot.subsystems.utils.telemetrymgr;

public class SwerveBot {

  //subsystem instances
  private final swervemain swervedrive;
  private final telemetrymgr telemetry; 
  private final controllermgr controller;

  public SwerveBot() {

    //subsytem declarations
    swervedrive = new swervemain();  
    telemetry = new telemetrymgr(swervedrive);
    controller = new controllermgr();
    
    //default command for swervedrive
    swervedrive.setDefaultCommand(new RunCommand(() -> 
      swervedrive.drive(
        controller.velocities()[0],
        controller.velocities()[1],
        controller.velocities()[2],
        controller.heading(),
        false),
        swervedrive));
  }

  public void getTeleopCommands() {

    //zero gyro when left stick button is pressed
    controller.drive_controller.leftStick().onTrue(swervedrive.resetGyro());
  }

  public void telemetry() {
    telemetry.swervemain(); //posts swervemain data
    telemetry.swervemodule();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
