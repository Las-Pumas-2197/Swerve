// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**Class used to transform driver controller axis data into usable speeds and functions for the driver.*/
public class drivercontroller extends SubsystemBase {

  //instances for objects
  public final CommandXboxController xbox; //make public so functions can be addressed
  private static final double pi = Math.PI;
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
  private static final double axispow = 2; //raise axis to the power of this to get finer control, higher is more progressive

  //vars for misc usage
  private double LXaxis;
  private double LYaxis;
  private double LZaxis;
  private double heading;

  //you know the deal
  public drivercontroller(int port) {
    xbox = new CommandXboxController(port);
    Xslew = new SlewRateLimiter(Xrate);
    Yslew = new SlewRateLimiter(Yrate);
    Zslew = new SlewRateLimiter(Zrate);
  }

  /**Array containing field-referenced velocities generated from transformed controller axis data.
   * @param maxlinspeed Maximum linear speed in m/s.
   * @param maxrotspeed Maximum rotational speed in rads/s.
   * @return The array.
   * @apiNote 0 = Xspeed
   * @apiNote 1 = Yspeed
   * @apiNote 2 = Zrot
   */
  public double[] velocities(double maxlinspeed, double maxrotspeed) {
    return new double[] {
      Xslew.calculate(
        MathUtil.applyDeadband(
          -Math.pow(LYaxis, axispow) * maxlinspeed, Xdeadband, maxlinspeed)),
      Yslew.calculate(
        MathUtil.applyDeadband(
          -Math.pow(LXaxis, axispow) * maxlinspeed, Ydeadband, maxlinspeed)),
      Zslew.calculate(
        MathUtil.applyDeadband(
          -Math.pow(LZaxis, axispow) * maxlinspeed, Zdeadband, maxrotspeed)),
    };
  }

  /**Use to inject driver controller into robotcontainer.*/
  public CommandXboxController xboxController() {
    return xbox;
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
    LXaxis = xbox.getLeftX();
    LYaxis = xbox.getLeftY();
    LZaxis = xbox.getLeftTriggerAxis() - xbox.getRightTriggerAxis();
  }
}
