// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class drivercontrols extends SubsystemBase {

  //instances for objects
  private final XboxController xbox;
  private static final double pi = Math.PI;
  private final SlewRateLimiter Xslew;
  private final SlewRateLimiter Yslew;
  private final SlewRateLimiter Zslew;

  //parameters
  private static final double Xrate = 2.5; //m/s^2
  private static final double Yrate = 2.5; //m/s^2
  private static final double Zrate = 1*pi; //rads/s^2
  private static final double Xdeadband = 0.1;
  private static final double Ydeadband = 0.1;
  private static final double Zdeadband = 0.05;
  private static final double headingincrement = 0.1;

  //vars for usage
  private double LXaxis;
  private double LYaxis;
  private double LZaxis;
  private double heading;

  //you know the deal
  public drivercontrols(int port) {
    xbox = new XboxController(port);
    Xslew = new SlewRateLimiter(Xrate);
    Yslew = new SlewRateLimiter(Yrate);
    Zslew = new SlewRateLimiter(Zrate);
  }

  /**Array containing field-referenced velocities generated from transformed controller L stick axis data.
   * @param maxlinspeed Maximum linear speed in m/s.
   * @param maxrotspeed Maximum rotational speed in rads/s.
   * @return The array.
   */
  public double[] velocities(double maxlinspeed, double maxrotspeed) {
    return new double[] {
      Xslew.calculate(
        MathUtil.applyDeadband(
          -Math.pow(LYaxis, 2) * maxlinspeed, Xdeadband, maxlinspeed)),
      Yslew.calculate(
        MathUtil.applyDeadband(
          -Math.pow(LXaxis, 2) * maxlinspeed, Ydeadband, maxlinspeed)),
      Zslew.calculate(
        MathUtil.applyDeadband(
          -Math.pow(LZaxis, 2) * maxlinspeed, Zdeadband, maxrotspeed)),
    };
  }

  /**Array containing pertinate button states. Currently ID 0 = Left Bumper, ID 1 = Left Stick.
   * @return Array of bools containing button states.
   */
  public boolean[] buttons() {
    return new boolean[] {
      xbox.getLeftBumper(),
      xbox.getLeftStickButton()
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
    LXaxis = xbox.getLeftX();
    LYaxis = xbox.getLeftY();
    LZaxis = xbox.getLeftTriggerAxis() - xbox.getRightTriggerAxis();
  }
}
