// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class drivercontrols extends SubsystemBase {

  private final XboxController xbox;
  private static final double pi = Math.PI;
  private final SlewRateLimiter Xslew;
  private final SlewRateLimiter Yslew;
  private final SlewRateLimiter Zslew;
  private static final double Xrate = 2.5; //m/s^2
  private static final double Yrate = 2.5; //m/s^2
  private static final double Zrate = 1*pi; //rads/s^2
  private static final double Xdeadband = 0.1;
  private static final double Ydeadband = 0.1;
  private static final double Zdeadband = 0.05;
  private static final double headingincrement = 0.1; 
  private double LXaxis;
  private double LYaxis;
  private double LZaxis;
  private double heading;

  public drivercontrols(int port) {
    xbox = new XboxController(port);
    Xslew = new SlewRateLimiter(Xrate);
    Yslew = new SlewRateLimiter(Yrate);
    Zslew = new SlewRateLimiter(Zrate);
  }

  public double[] velocities(double maxlinspeed, double maxrotspeed) {
    return new double[] {
      Xslew.calculate(MathUtil.applyDeadband(-LYaxis, Xdeadband, maxlinspeed)),
      Yslew.calculate(MathUtil.applyDeadband(-LXaxis, Ydeadband, maxlinspeed)),
      Zslew.calculate(MathUtil.applyDeadband(-LZaxis, Zdeadband, maxrotspeed)),
    };
  }

  public boolean[] buttons() {
    return new boolean[] {
      xbox.getLeftStickButton()
    };
  } 

  public double heading() {
      if (Math.abs(LZaxis) > Zdeadband) {
        heading = MathUtil.angleModulus(heading + (LZaxis * pi * headingincrement));
      }
    return heading;
  }

  @Override
  public void periodic() {
    LXaxis = xbox.getLeftX();
    LYaxis = xbox.getLeftY();
    LZaxis = xbox.getLeftTriggerAxis() - xbox.getRightTriggerAxis();
  }
}
