// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.swervemain;
import frc.robot.subsystems.drivetrain.swerveconfig.mainconfig;

public class trajectorymgr extends SubsystemBase {

  private final swervemain swervedrive;
  private final TrajectoryConfig cfg_auto;
  private final TrajectoryConfig cfg_teleop;
  private final PIDController pid_Xpos;
  private final PIDController pid_Ypos;
  private final ProfiledPIDController pid_heading;
  private final SimpleMotorFeedforward ff_heading;

  public trajectorymgr(swervemain drivetrain) {

    //injected drivetrain subsystem
    swervedrive = drivetrain;

    //trajectory configs
    cfg_auto = new TrajectoryConfig(
      mainconfig.maxlinvel_auto,
      mainconfig.maxlinacl_auto)
      .setKinematics(
        mainconfig.kinematics);
    cfg_teleop = new TrajectoryConfig(
      mainconfig.maxlinvel_teleop,
      mainconfig.maxlinacl_teleop)
        .setKinematics(
          mainconfig.kinematics);
    
    //PID controllers for trajectory manager
    pid_Xpos = new PIDController(
      0, 
      0, 
      0);
    pid_Ypos = new PIDController(
      0,
      0,
      0);
    pid_heading = new ProfiledPIDController(
      mainconfig.pid_headingkP,
      0,
      mainconfig.pid_headingkD,
      mainconfig.heading_constraints);
    ff_heading = new SimpleMotorFeedforward(
      mainconfig.ff_headingkS,
      mainconfig.ff_headingkV,
      mainconfig.ff_headingkA);
  }

  @Override
  public void periodic() {}
}
