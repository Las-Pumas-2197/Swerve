// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.swervemain;
import frc.robot.subsystems.drivetrain.swerveconfig.mainconfig;
import frc.robot.subsystems.utils.drivercontroller;
import frc.robot.subsystems.utils.telemetry;

public class SwerveBot {

  //subsystem instances
  private final swervemain swervedrive;
  private final drivercontroller drivercontrols;
  private final telemetry telemetrymanager; 
  private final CommandXboxController xboxController;

  //drivercontrols velocities array
  private final double[] velocities;

  public SwerveBot() {

    //subsytem declarations
    swervedrive = new swervemain();
    drivercontrols = new drivercontroller(0);
    telemetrymanager = new telemetry(swervedrive);

    //injected objects
    xboxController = drivercontrols.xboxController();

    //velocities array
    velocities = drivercontrols.velocities(mainconfig.maxlinvel_teleop, mainconfig.maxrotvel_teleop);
  }

  public void getTeleopCommands() {

    //zero gyro when A is pressed
    xboxController.a().onTrue(swervedrive.resetGyro());

    //drive
    swervedrive.drive(
      velocities[0],
      velocities[1],
      velocities[2],
      drivercontrols.heading(),
      true);
  }

  public void telemetry() {
    telemetrymanager.swervemain(); //posts swervemain data
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
