// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.swerveconfig.mainconfig;

/**Class used to transform driver controller axis data into usable speeds and functions for the driver.*/
public class controllermgr extends SubsystemBase {

  //instances for objects
  public final CommandXboxController drive_controller; //make public so triggers and axis data can be accessed
  private final SlewRateLimiter Xslew;
  private final SlewRateLimiter Yslew;
  private final SlewRateLimiter Zslew;

  //parameters
  private static final double Xrate = 1000; //m/s^2 2.5 stupid high number to make instant for tuning
  private static final double Yrate = 1000; //m/s^2 2.5
  private static final double Zrate = 1000; //rads/s^2 1*pi
  private static final double Xdeadband = 0.1;
  private static final double Ydeadband = 0.1;
  private static final double Zdeadband = 0.05;
  private static final double headingincrement = 0.1;
  private static final double axispow = 2; //raise axis by this power, higher is more progressive

  //vars for misc usage
  private static final double pi = Math.PI;
  private static final double rad = 2*pi;
  private final double maxlinvel;
  private final double maxrotvel;
  private double LXaxis;
  private double LYaxis;
  private double LZaxis;
  private double heading;

  /**Controller manager for robot. Has some utils for transforming axis data and*/
  public controllermgr() {
    drive_controller = new CommandXboxController(0);
    Xslew = new SlewRateLimiter(Xrate);
    Yslew = new SlewRateLimiter(Yrate);
    Zslew = new SlewRateLimiter(Zrate);
    maxlinvel = mainconfig.maxlinvel_teleop;
    maxrotvel = mainconfig.heading_maxvel;
  }

  /**Generates and array containing field-referenced velocities generated from transformed controller axis data.
   * @return The array.
   * @apiNote 0 = Xspeed
   * @apiNote 1 = Yspeed
   * @apiNote 2 = Zrot
   */
  public double[] velocities() {
    return new double[] {
      Xslew.calculate(
        MathUtil.applyDeadband(
          (Math.pow(LYaxis, axispow) * Math.signum(LYaxis)) * maxlinvel, Xdeadband, maxlinvel)),
      Yslew.calculate(
        MathUtil.applyDeadband(
          (Math.pow(LXaxis, axispow) * Math.signum(LXaxis)) * maxlinvel, Ydeadband, maxlinvel)),
      Zslew.calculate(
        MathUtil.applyDeadband(
          (Math.pow(LZaxis, axispow) * Math.signum(LZaxis)) * maxrotvel, Zdeadband, maxrotvel)),
    };
  }

  /**Double containing the desired heading generated from transformed controller axis data.
   * @return Heading desired double.
   */
  public double heading() {
      if (Math.abs(LZaxis) > Zdeadband) {
        heading = MathUtil.angleModulus(heading + (LZaxis * pi * headingincrement));
      }
    return heading;
  }

  @Override
  public void periodic() {

    //stuff and things
    LXaxis = drive_controller.getLeftX();
    LYaxis = drive_controller.getLeftY();
    LZaxis = drive_controller.getLeftTriggerAxis() - drive_controller.getRightTriggerAxis();
  }
}
