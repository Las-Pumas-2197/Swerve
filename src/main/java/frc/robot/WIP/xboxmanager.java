// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.WIP;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class xboxmanager extends SubsystemBase {

  //instantiate xbox controller object
  private final XboxController xbox;

  //arrays
  private boolean[] buttonarray;
  private double[] axisarray;

  //pi
  private static final double pi = Math.PI;

  //instantiate slews and deadbands for transformations
  private final SlewRateLimiter Xslew;
  private final SlewRateLimiter Yslew;
  private final SlewRateLimiter Zslew;
  private static final double Xrate = 2.5; //in m/s^2
  private static final double Yrate = 2.5; //in m/s^2
  private static final double Zrate = 1*pi; //in rads/s^2
  private static final double Xdeadband = 0.1; //% of axis
  private static final double Ydeadband = 0.1; //% of axis
  private static final double Zdeadband = 0.05; //% of axis

  //speeds from transformed axis data
  private double Xspeed;
  private double Yspeed;
  private double Zrot;

  //heading values
  private double heading;
  private static final double headingincrement = 0.05*pi; //mutliplier for increment of heading value

  /** Xbox controller management class. Also includes some transformations for holonomic drives.
   * @param port Port of the controller in question.
   */
  public xboxmanager(int port) {
    
    //controller construction
    xbox = new XboxController(port);

    //slew limiters for axis transformations
    Xslew = new SlewRateLimiter(Xrate);
    Yslew = new SlewRateLimiter(Yrate);
    Zslew = new SlewRateLimiter(Zrate);
  }

  /** Returns speeds from transformations in xboxmanager. Can be used with any holonomic drive.
   * Pass in maximum linear and rotational speeds to scale the resultant speeds properly.
   * @param MaxSpeedMeters Maximum linear speed of the drivetrain. In rads/s.
   * @param MaxRotRads Maximum rotational speed of drivetrain. In rads/s.
   * @return An array containing linear X and Y speeds in m/s, and a Z speed in rads/s.
   */
  public double[] holonomicspeeds(double MaxSpeedMeters, double MaxRotRads) {

    //axis creation
    double Xaxisraw = -xbox.getLeftY();
    double Yaxisraw = -xbox.getLeftX();
    double Zaxisraw = xbox.getLeftTriggerAxis() - xbox.getRightTriggerAxis();
    
    //axis transformation to speeds
    Xspeed = Xslew.calculate(MathUtil.applyDeadband(Xaxisraw, Xdeadband, MaxSpeedMeters));
    Yspeed = Yslew.calculate(MathUtil.applyDeadband(Yaxisraw, Ydeadband, MaxSpeedMeters));
    Zrot = Zslew.calculate(MathUtil.applyDeadband(Zaxisraw, Zdeadband, MaxRotRads));

    return new double[] {
      Xspeed,
      Yspeed,
      Zrot
    };
  }

  public double headingrads() {

    //create Z axis from trigger data
    double Zaxis = (xbox.getLeftTriggerAxis() - xbox.getRightTriggerAxis())*pi;

    //increment heading based on trigger inputs
    if (Math.abs(Zaxis) > Zdeadband) {
      heading = MathUtil.angleModulus(heading + (headingincrement * Zaxis));
    }

    //zero heading when A is pressed
    if (xbox.getAButton()) {
      heading = 0;
    }

    return heading;
  }

  @Override
  public void periodic() {}
}
