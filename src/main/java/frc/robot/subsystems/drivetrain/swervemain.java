// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.swerveconfig.mainconfig;
import frc.robot.subsystems.drivetrain.swerveconfig.moduleconfig;

/**Main class for swerve drive subsystem.*/
public class swervemain extends SubsystemBase {
  
  //module instances
  private final swervemodule FLmodule;
  private final swervemodule FRmodule;
  private final swervemodule RLmodule;
  private final swervemodule RRmodule;

  //PID and FF for heading
  private final ProfiledPIDController pid_heading;
  private final SimpleMotorFeedforward ff_heading;

  //gyroscope
  private final Pigeon2 gyro;

  //kinematics and odometry
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  //pi, again
  private static final double pi = Math.PI;

  //misc vars for telemetry and calculations
  private double Xspeed_des;
  private double Yspeed_des;
  private double Zrot_des;
  private double Zinput; //used to pass either Zrot or PIDF output to IK calculations
  private double heading_pidout;
  private double heading_ffout;
  private double heading_des;
  private double heading_act;
  private double heading_err;

  /**Main subsytem for swerve drive. Swerve modules are subclassed off this class. Written in command.*/
  public swervemain() {

    //declare modules
    FLmodule = new swervemodule(mainconfig.FLazimuthID, mainconfig.FLdriveID, mainconfig.FLoffset);
    FRmodule = new swervemodule(mainconfig.FRazimuthID, mainconfig.FRdriveID, mainconfig.FRoffset);
    RLmodule = new swervemodule(mainconfig.RLazimuthID, mainconfig.RLdriveID, mainconfig.RLoffset);
    RRmodule = new swervemodule(mainconfig.RRazimuthID, mainconfig.RRdriveID, mainconfig.RRoffset);

    //heading PID and FF
    pid_heading = new ProfiledPIDController(
      mainconfig.pid_headingkP,
      0, //not used
      mainconfig.pid_headingkD,
      mainconfig.heading_constraints);
    ff_heading = new SimpleMotorFeedforward(
      mainconfig.ff_headingkS, 
      mainconfig.ff_headingkV, 
      mainconfig.ff_headingkA);

    //gyroscope
    gyro = new Pigeon2(mainconfig.gyroID);
    gyro.reset();

    //kinematics, note that IK calculations will return an array with states for each module in the order they were declared
    kinematics = mainconfig.kinematics; //FL, FR, RL, RR order

    //odometry, used to calculate approximate pose of robot
    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(heading_act), getPositions());
  }

  /**Returns current module states in an array. Ordered as FL, FR, RL, RR.*/
  public SwerveModuleState[] getActStates() {
    return new SwerveModuleState[] {
      FLmodule.getState(),
      FRmodule.getState(),
      RLmodule.getState(),
      RRmodule.getState()
    };
  }

  /**Returns current module positions in an array. Ordered as FL, FR, RL, RR.*/
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      FLmodule.getPosition(),
      FRmodule.getPosition(),
      RLmodule.getPosition(),
      RRmodule.getPosition()
    };
  }

  /**Returns the current calculated pose of the robot.*/
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**Resets the gyroscope of the robot. */
  public InstantCommand resetGyro() {
    return new InstantCommand(() -> gyro.reset());
  }

  /**Returns a 2-dimensional table of doubles containing telemetry data. First integer is the module to address, second
   * is the data point to return.
   * <p> First integer:
   * <ul>
   *    <li> 0 = FLmodule
   *    <li> 1 = FRmodule
   *    <li> 2 = RLmodule
   *    <li> 3 = RRmodule
   * </ul>
   * <p> Second integer:
   * <ul>
   *    <li> 0 = azimuthpos_des
   *    <li> 1 = azimuthpos_act
   *    <li> 2 = azimuth_err
   *    <li> 3 = azimuth_pidout
   *    <li> 4 = azimuth_ffout
   *    <li> 5 = azimuth_volts
   *    <li> 6 = drivevel_des
   *    <li> 7 = drivevel_act
   *    <li> 8 = drive_err
   *    <li> 9 = drive_ffout
   *    <li> 10 = drive_volts
   * </ul>
  */
  public double[][] getModuleTelemetry() {
    return new double[][] {
      FLmodule.getTelemetry(),
      FRmodule.getTelemetry(),
      RLmodule.getTelemetry(),
      RRmodule.getTelemetry()
    };
  }

  /**Returns an array containing pertinate drivetrain data.
   * <p> Note that if HeadingCL = true, Zinput = heading_pidout + heading_ffout. If false, Zinput = Zrot.
   * <ul>
   *    <li> 0 = Xspeed_des
   *    <li> 1 = Yspeed_des
   *    <li> 2 = Zinput
   *    <li> 3 = Zrot
   *    <li> 4 = heading_act
   *    <li> 5 = heading_des
   *    <li> 6 = heading_pidout
   *    <li> 7 = heading_ffout
   * </ul>
  */
  public double[] getMainTelemetry() {
    return new double[] {
      Xspeed_des,
      Yspeed_des,
      Zinput,
      Zrot_des,
      heading_act,
      heading_des,
      heading_pidout,
      heading_ffout
    };
  }

  //Note that drive() must be in a void method or will trip illegal arg for calling multiple commands from the same subsys,
  //even though technically are not the same subsystem since they are different instances, will investigate bug

  /** Operate drivetrain using passed speeds. If using with a trajectory generator, do NOT use closed loop
   * heading control, as that is handled by the trajectory controller in that state.
  * @param Xspeed Linear X speed in m/s.
  * @param Yspeed Linear Y speed in m/s.
  * @param Zrot Rotational Z speed in rads/s.
  * @param HeadingDesired Desired heading in rads. Must be wrapped -pi to pi.
  * @param HeadingCL To operate with heading in CL or OL.
  */
  public void drive(double Xspeed, double Yspeed, double Zrot, double HeadingDesired, boolean HeadingCL) {

    //write inputs to internal vars
    Xspeed_des = Xspeed;
    Yspeed_des = Yspeed;
    Zrot_des = Zrot;
    heading_des = HeadingDesired;
    heading_act = MathUtil.angleModulus((-gyro.getAngle()/180)*pi);
    heading_err = MathUtil.angleModulus(heading_des - heading_act); //modulo used to wrap when beyond [-pi, pi] interval

    //conditional for Z input to IK calcs
    if (HeadingCL) {
      heading_ffout = (ff_heading.calculate(heading_err) / moduleconfig.maxvolts); //convert to rads
      heading_pidout = pid_heading.calculate(heading_act, heading_des);
      Zinput = heading_pidout + heading_ffout;
    } else {
      Zinput = Zrot_des;
    }

    //construct chassisspeeds instance for IK calcs and discretize
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(Xspeed, Yspeed, Zinput, new Rotation2d(heading_act));
    ChassisSpeeds.discretize(speeds, 0.02);

    //IK calcs, desaturate, and optimize
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, moduleconfig.drive_maxvel); //make this maximum achievable velocity
    SwerveModuleState[] optimizedstates = new SwerveModuleState[] {
      SwerveModuleState.optimize(states[0], FLmodule.getState().angle),
      SwerveModuleState.optimize(states[1], FRmodule.getState().angle),
      SwerveModuleState.optimize(states[2], RLmodule.getState().angle),
      SwerveModuleState.optimize(states[3], RRmodule.getState().angle)
    };

    //multiply speed desired by cosine of angle error
    optimizedstates[0].speedMetersPerSecond *= Math.cos(FLmodule.getTelemetry()[2]);
    optimizedstates[1].speedMetersPerSecond *= Math.cos(FRmodule.getTelemetry()[2]);
    optimizedstates[2].speedMetersPerSecond *= Math.cos(RLmodule.getTelemetry()[2]);
    optimizedstates[3].speedMetersPerSecond *= Math.cos(RRmodule.getTelemetry()[2]);

    /**
    //alternate method to derate speed based on module error, takes highest error and multiplies all speeds by inverse
    //checks for maximum error, get inverse, multiple speed by the multiplier
    double errormult = 
      Math.cos(
        Math.max(
          Math.max(
            FLmodule.getTelemetry()[2],
            FRmodule.getTelemetry()[2]),
          Math.max(
            RLmodule.getTelemetry()[2],
            RRmodule.getTelemetry()[2])));
    optimizedstates[0].speedMetersPerSecond *= errormult;
    optimizedstates[1].speedMetersPerSecond *= errormult;
    optimizedstates[2].speedMetersPerSecond *= errormult;
    optimizedstates[3].speedMetersPerSecond *= errormult;
    */

    //write states
    FLmodule.setState(optimizedstates[0]); //FL
    FRmodule.setState(optimizedstates[1]); //FR
    RLmodule.setState(optimizedstates[2]); //RL
    RRmodule.setState(optimizedstates[3]); //RR
  }

  @Override
  public void periodic() {

    //update odometry 
    odometry.update(new Rotation2d(heading_act), new SwerveModulePosition[] {
      FLmodule.getPosition(),
      FRmodule.getPosition(),
      RLmodule.getPosition(),
      RRmodule.getPosition()
    });
  }
}
