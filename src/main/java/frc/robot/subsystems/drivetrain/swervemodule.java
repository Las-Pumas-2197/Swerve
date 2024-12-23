// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.swerveconfig.moduleconfig;

public class swervemodule extends SubsystemBase {
  
  //motor instances
  private final CANSparkMax m_azimuth;
  private final CANSparkFlex m_drive;

  //encoder instances
  private final AbsoluteEncoder enc_azimuth;
  private final RelativeEncoder enc_drive;

  //PID and FF controllers
  private final ProfiledPIDController pid_azimuth;
  private final SimpleMotorFeedforward ff_azimuth;
  private final SimpleMotorFeedforward ff_drive;

  //mmm pi very gud
  private static final double pi = Math.PI;

  //module offset
  private final double moduleoffset;

  //misc vars for telemetry
  private double azimuth_posdes;
  private double azimuth_posact;
  private double azimuth_err;  
  private double azimuth_pidout;
  private double azimuth_ffout;
  private double azimuth_volts;
  private double drive_veldes; //used for OL control when not in CL pos control
  private double drive_velact;
  private double drive_posact;
  private double drive_err;
  private double drive_ffout;
  private double drive_volts;

  /**Swerve module constructor. Feed just the node IDs of the drive and azimuth nodes, as well as the offset of the module
   * (dependent on the position the module is in).
   * @param azimuthID Node ID of the CANSparkMax that controls azimuth.
   * @param driveID Node ID of the CANSparkFlex that controls drive.
   * @param offset The amount to offset the absolute encoder by. Depends on module position of the drivetrain.
  */
  public swervemodule(int azimuthID, int driveID, double offset) {
    
    //declare motors
    m_azimuth = new CANSparkMax(azimuthID, MotorType.kBrushless);
    m_drive = new CANSparkFlex(driveID, MotorType.kBrushless);

    //drive motor inversion due to MAXswerve design
    m_drive.setInverted(true);

    //declare encoders
    enc_azimuth = m_azimuth.getAbsoluteEncoder();
    enc_drive = m_drive.getEncoder();

    //setting module offset to internal var
    moduleoffset = offset;

    //conversion factors
    enc_azimuth.setVelocityConversionFactor(moduleconfig.con_azimuth_velfactor);
    enc_azimuth.setPositionConversionFactor(moduleconfig.con_azimuth_posfactor);
    enc_drive.setVelocityConversionFactor(moduleconfig.con_drive_velfactor);
    enc_drive.setPositionConversionFactor(moduleconfig.con_drive_posfactor);

    //PID and FF controllers for turn
    pid_azimuth = new ProfiledPIDController(
      moduleconfig.pid_azimuth_kP,
      moduleconfig.pid_azimuth_kI,
      moduleconfig.pid_azimuth_kD,
      moduleconfig.azimuth_constraints);
    ff_azimuth = new SimpleMotorFeedforward(
      moduleconfig.ff_azimuth_kS,
      moduleconfig.ff_azimuth_kV,
      moduleconfig.ff_azimuth_kA); //may omit if not needed

    //FF controller for drive
    ff_drive = new SimpleMotorFeedforward(
      moduleconfig.ff_drive_kS,
      moduleconfig.ff_drive_kV,
      moduleconfig.ff_drive_kA);

    //PID wrapping for azimuth
    pid_azimuth.enableContinuousInput(-pi, pi);
  }
  
    /**Returns the current state of the module.*/
  public SwerveModuleState getState() {
    return new SwerveModuleState(drive_velact, new Rotation2d(azimuth_posact));
  }

  /**Polls the current distance recorded by the drive encoder, returns a polar with distance and module angle.*/
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(drive_posact, new Rotation2d(azimuth_posact));
  }

  /**Resets the recorded position of the drive encoder.*/
  public InstantCommand resetPosition() {
    return new InstantCommand(() -> enc_drive.setPosition(0));
  }

  /**Sets the desired state of the module.
   * @param state The desired state of the module.
  */
  public void setState(SwerveModuleState state) {

    //write desired state data to internal vars for calculation
    azimuth_posdes = state.angle.getRadians(); //should be wrapped in interval [-pi, pi]
    drive_veldes = state.speedMetersPerSecond; //used for open loop control

    //calculate FFs and PIDs, PID output needs converted to volts but FFs are already in volts
    azimuth_pidout = pid_azimuth.calculate(azimuth_posact, azimuth_posdes);
    azimuth_ffout = ff_azimuth.calculate(azimuth_err);
    drive_ffout = ff_drive.calculate(drive_veldes);

    //run command using above calcs, azimuth PID must be normalized to units of volts
    azimuth_volts = ((azimuth_pidout / moduleconfig.azimuth_maxvel) * moduleconfig.maxvolts) + azimuth_ffout;
    drive_volts = drive_ffout;
    //m_azimuth.setVoltage(azimuth_volts);
    //m_drive.setVoltage(drive_volts);
  }

  /**Returns an array of doubles containing the telemetry of the module. Data is as follows:
   * <ul>
   *    <li> 0 = azimuth_posdes
   *    <li> 1 = azimuth_posact
   *    <li> 2 = azimuth_err
   *    <li> 3 = azimuth_pidout
   *    <li> 4 = azimuth_ffout
   *    <li> 5 = azimuth_volts
   *    <li> 6 = drive_veldes
   *    <li> 7 = drive_velact
   *    <li> 8 = drive_err
   *    <li> 9 = drive_ffout
   *    <li> 10 = drive_volts
   * </ul>
  */
  public double[] getTelemetry() {
    return new double[] {
      azimuth_posdes,
      azimuth_posact,
      azimuth_err,
      azimuth_pidout,
      azimuth_ffout,
      azimuth_volts,
      drive_veldes,
      drive_velact,
      drive_err,
      drive_ffout,
      drive_volts
    };
  }

  @Override
  public void periodic() {

    //apply offset to encoder and write actual data to internal vars
    //possibly run offsets in swervemain in later iterations
    azimuth_posact = MathUtil.angleModulus(enc_azimuth.getPosition() + moduleoffset);
    drive_velact = enc_drive.getVelocity();
    drive_posact = enc_drive.getPosition();

    //calculate error using maths
    azimuth_err = MathUtil.angleModulus(azimuth_posdes - azimuth_posact);
    drive_err = drive_veldes - drive_velact;
  }
}
