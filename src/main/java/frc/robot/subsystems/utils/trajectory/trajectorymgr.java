// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.utils.trajectory;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
  private Trajectory trajectory;

  private static final double pi = Math.PI;

  public trajectorymgr(swervemain drivetrain) {

    //injected drivetrain subsys
    swervedrive = drivetrain;

    //trajectory configs
    cfg_auto = new TrajectoryConfig(
      mainconfig.maxlinvel_auto,
      mainconfig.maxlinacl_auto)
      .setKinematics(mainconfig.kinematics);
    cfg_teleop = new TrajectoryConfig(
      mainconfig.maxlinvel_teleop,
      mainconfig.maxlinacl_teleop)
      .setKinematics(mainconfig.kinematics);

    //PID controllers for trajectory generator
    pid_Xpos = new PIDController(
      trajectoryconfig.pid_trajkP, 
      0, 
      0);
    pid_Ypos = new PIDController(
      trajectoryconfig.pid_trajkP,
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

    //heading PID controller options
    pid_heading.enableContinuousInput(-pi, pi);
  }

  public Command followtrajectoryteleop(Pose2d startpose, Translation2d[] waypoints, Pose2d endpose) {

    //generate trajectory from 
    trajectory = TrajectoryGenerator.generateTrajectory(
      startpose,
      List.of(
        waypoints[0],
        waypoints[1]),
      endpose,
      cfg_teleop);

    //trajectory follower command
    SwerveControllerCommand trajectorycommand = new SwerveControllerCommand(
      trajectory, //the trajectory generated above
      swervedrive::getPose, //Pose2d consumer functional interface for supplier
      mainconfig.kinematics, //kinematics object
      pid_Xpos, //x position pid controller
      pid_Ypos, //y position pid controller
      pid_heading, //heading pid controller
      swervedrive::setStates, //SwerveModuleState[] supplier functional interface for consumer
      swervedrive);

    return Commands.sequence(
      new InstantCommand(() -> swervedrive.resetPose(trajectory.getInitialPose())),
      trajectorycommand,
      new InstantCommand(() -> swervedrive.drive(0, 0, 0, 0, false)));
  }

  @Override
  public void periodic() {}
}
