// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.driverinputs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HIDs extends SubsystemBase {

  //construct joystick and xbox controller
  public final Joystick joystick;
  private final XboxController xbox;

  //construct joystick raw data
  private double joystickX;
  private double joystickY;
  private double joystickZ;

  //deadband values for joystick
  private double joystickdeadbandX = 0.2;
  private double joystickdeadbandY = 0.2;
  private double joystickdeadbandZ = 0.5;

  //xbox raw data
  private double xboxX;
  private double xboxY;
  private double xboxZ;

  //xbox deadband values
  private double xboxdeadbandX = 0.1;
  private double xboxdeadbandy = 0.1;
  private double xboxdeadbandZ = 0.5;

  //max output velocities
  private double maxspeedmps = 3;
  private double maxrotspeedrads = Math.PI;

  //desired heading value
  private double headingdesired;

  //slew limitier objects
  SlewRateLimiter Xaxisslew = new SlewRateLimiter(2.5);
  SlewRateLimiter Yaxisslew = new SlewRateLimiter(2.5);
  SlewRateLimiter Zaxisslew = new SlewRateLimiter(0.25*Math.PI);

  //slew limier objects for xbox
  SlewRateLimiter Xaxisslewxbox = new SlewRateLimiter(2.5);
  SlewRateLimiter Yaxisslewxbox = new SlewRateLimiter(2.5);

  /** HID class for running joystick data. */
  public HIDs() {
    joystick  = new Joystick(0);
    xbox = new XboxController(1);
  }

  /**
   * Get X, Y, and Z axis data from joystick.
   * @return An array containing X Y and Z doubles.
   */
  public double[] getRawJoystickData() {
    return new double[] {
      Math.cbrt(joystickX)*maxspeedmps,
      Math.cbrt(joystickY)*maxspeedmps,
      Math.cbrt(joystickZ)*maxrotspeedrads
    };
  }

  /**
   * Returns desired velocities from raw joystick data.
   * @return An array contained X and Y speeds in m/s, and Z rot in rads/s.
   */
  public double[] getJoystickVelocityData() {
    
    //transform joystick inputs into velocities
    double Xspeed = MathUtil.applyDeadband(-joystickY, joystickdeadbandY)*maxspeedmps;
    double Yspeed = MathUtil.applyDeadband(-joystickX, joystickdeadbandX)*maxspeedmps;
    double Zspeed = MathUtil.applyDeadband(-joystickZ, joystickdeadbandZ)*maxrotspeedrads;

    return new double[] {
      Xspeed,
      Yspeed,
      Zspeed
    };
  }

  public double[] getXboxVelocityData() {
    return new double[] {
      MathUtil.applyDeadband(-xboxY, 0.1)*maxspeedmps,
      MathUtil.applyDeadband(-xboxX, 0.1)*maxrotspeedrads,
      MathUtil.applyDeadband(xboxZ, 0.1)*maxrotspeedrads,
      headingdesired
    };
  }

  @Override
  public void periodic() {

    //get joystick axis data and slew
    joystickX = Xaxisslew.calculate(joystick.getX());
    joystickY = Yaxisslew.calculate(joystick.getY());
    joystickZ = Zaxisslew.calculate(joystick.getZ());

    //get xbox axis data and slew
    xboxX = Xaxisslewxbox.calculate(xbox.getLeftX());
    xboxY = Yaxisslewxbox.calculate(xbox.getLeftY());
    xboxZ = 0;

    //conditional for heading control
    if (xbox.getRightBumper() == true) {
      headingdesired = headingdesired - 0.05;
    }
    if (xbox.getLeftBumper() == true) {
      headingdesired = headingdesired + 0.05;
    }

  }
}
