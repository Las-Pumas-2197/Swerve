// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

public class swervedrive extends SubsystemBase {

  //construct module actual values
  private double FLdrivevelactual;
  private double FLturnposactual;
  private double FRdrivevelactual;
  private double FRturnposactual;
  private double RRdrivevelactual;
  private double RRturnposactual;
  private double RLdrivevelactual;
  private double RLturnposactual;

  //construct module desired values
  private double FLdriveveldes;
  private double FLturnposdes;
  private double FRdriveveldes;
  private double FRturnposdes;
  private double RRdriveveldes;
  private double RRturnposdes;
  private double RLdriveveldes;
  private double RLturnposdes;

  //construct heading values
  private double headingactual;
  private double headingdes;

  //construct motor controller objects
  private final CANSparkFlex FLdrive;
  private final CANSparkFlex FRdrive;
  private final CANSparkFlex RRdrive;
  private final CANSparkFlex RLdrive;
  private final CANSparkMax FLturn;
  private final CANSparkMax FRturn;
  private final CANSparkMax RRturn;
  private final CANSparkMax RLturn;

  //construct encoder objects
  private final RelativeEncoder FLdriveencoder;
  private final AbsoluteEncoder FLturnencoder;
  private final RelativeEncoder FRdriveencoder;
  private final AbsoluteEncoder FRturnencoder;
  private final RelativeEncoder RRdriveencoder;
  private final AbsoluteEncoder RRturnencoder;
  private final RelativeEncoder RLdriveencoder;
  private final AbsoluteEncoder RLturnencoder;

  //pi var for convenience
  private static final double pi = Math.PI;

  //Construct Gyro Object
  private final Pigeon2 gyro;

  //construct kinematics object
  private static final double wheelbase = Units.inchesToMeters(26.5);
  private static final double trackwidth = Units.inchesToMeters(26.5);
  private final SwerveDriveKinematics kinematics;

  //PID loop construction
  private final ProfiledPIDController FLturnPID;
  private final ProfiledPIDController FRturnPID;
  private final ProfiledPIDController RLturnPID;
  private final ProfiledPIDController RRturnPID;
  private final ProfiledPIDController headingPID;

  //linear filters for encoders
  private final LinearFilter FLturnfilter;
  private final LinearFilter FRturnfilter;
  private final LinearFilter RLturnfilter;
  private final LinearFilter RRturnfilter;

  //PID output values for diagnostics
  private double FLturnPIDout;
  private double FRturnPIDout;
  private double RLturnPIDout;
  private double RRturnPIDout;
  private double headingPIDout;

  //trapezoidal constraints for turn PID
  private final TrapezoidProfile.Constraints turnprofile;
  private final TrapezoidProfile.Constraints headingprofile;

  //misc tuning parameters for PID loops, FFs, and other calcs
  private static final double turnPIDkP = 1.25;
  private static final double turnPIDkD = 0.35;
  private static final double turntol = 0.05*pi;
  private static final double headingPIDkP = 5;
  private static final double headingPIDkD = 0;
  private static final double headingtol = 0.05*pi;

  //velocity and voltage constraints
  private static final double maxappliedvoltage = 12;

  //velocity constraints
  private static final double maxspeedms = 5;
  private static final double maxrotspeedrads = 2*pi;

  private static final double maxconspeedrads = 4*pi;
  private static final double maxconaclrads = 8*pi;

  private static final double maxheadingspdrads = 4*pi;
  private static final double maxheadingaclrads = 4*pi;

  //conversion factors for velocity
  private static final double driveposconvfactor = (0.0762*pi / 4.71428); //meters
  private static final double drivevelconvfactor = (0.0762*pi / 4.71428) / 60; //m/s
  private static final double turnposconvfactor = 2*pi; //rads
  private static final double turnvelconvfactor = 2*pi; //rads/s

  //initializations for swervedrive
  public swervedrive() {
    //instantiate motor controllers
    FLdrive = new CANSparkFlex(4, MotorType.kBrushless);
    FRdrive = new CANSparkFlex(6, MotorType.kBrushless);
    RRdrive = new CANSparkFlex(8, MotorType.kBrushless);
    RLdrive = new CANSparkFlex(10, MotorType.kBrushless);
    FLturn = new CANSparkMax(3, MotorType.kBrushless);
    FRturn = new CANSparkMax(5, MotorType.kBrushless);
    RRturn = new CANSparkMax(7, MotorType.kBrushless);
    RLturn = new CANSparkMax(9, MotorType.kBrushless);

    //drive motor inversions
    FLdrive.setInverted(true);
    FRdrive.setInverted(true);
    RLdrive.setInverted(true);
    RRdrive.setInverted(true);

    //Instantiate Encoders
    FLdriveencoder = FLdrive.getEncoder();
    FRdriveencoder = FRdrive.getEncoder();
    RRdriveencoder = RRdrive.getEncoder();
    RLdriveencoder = RLdrive.getEncoder();
    FLturnencoder = FLturn.getAbsoluteEncoder();
    FRturnencoder = FRturn.getAbsoluteEncoder();
    RLturnencoder = RLturn.getAbsoluteEncoder();
    RRturnencoder = RRturn.getAbsoluteEncoder();

    //encoder options
    FLturnencoder.setPositionConversionFactor(turnposconvfactor);
    FRturnencoder.setPositionConversionFactor(turnposconvfactor);
    RLturnencoder.setPositionConversionFactor(turnposconvfactor);
    RRturnencoder.setPositionConversionFactor(turnposconvfactor);
    FLturnencoder.setVelocityConversionFactor(turnvelconvfactor);
    FRturnencoder.setVelocityConversionFactor(turnvelconvfactor);
    RLturnencoder.setVelocityConversionFactor(turnvelconvfactor);
    RRturnencoder.setVelocityConversionFactor(turnvelconvfactor);

    FLdriveencoder.setPositionConversionFactor(driveposconvfactor);
    FRdriveencoder.setPositionConversionFactor(driveposconvfactor);
    RLdriveencoder.setPositionConversionFactor(driveposconvfactor);
    RRdriveencoder.setPositionConversionFactor(driveposconvfactor);
    FLdriveencoder.setVelocityConversionFactor(drivevelconvfactor);
    FRdriveencoder.setVelocityConversionFactor(drivevelconvfactor);
    RLdriveencoder.setVelocityConversionFactor(drivevelconvfactor);
    RRdriveencoder.setVelocityConversionFactor(drivevelconvfactor);

    //instantiate gyroscope and reset
    gyro = new Pigeon2(20);
    gyro.reset();

    //instantiate kinematics object, passed in FL, FR, RL, RR order, with X offset first and Y offset second, NWU convention
    kinematics = new SwerveDriveKinematics(
      new Translation2d(wheelbase/2, trackwidth/2),
      new Translation2d(wheelbase/2, -trackwidth/2),
      new Translation2d(-wheelbase/2, trackwidth/2),
      new Translation2d(-wheelbase/2, -trackwidth/2)
    );

    //instantiate trapezoidal profile
    headingprofile = new TrapezoidProfile.Constraints(maxheadingspdrads, maxheadingaclrads);
    turnprofile = new TrapezoidProfile.Constraints(maxconspeedrads, maxconaclrads); //rads/s and rads/s^2

    //instantiate PID loops and FFs
    FLturnPID = new ProfiledPIDController(turnPIDkP, 0, turnPIDkD, turnprofile);
    FRturnPID = new ProfiledPIDController(turnPIDkP, 0, turnPIDkD, turnprofile);
    RLturnPID = new ProfiledPIDController(turnPIDkP, 0, turnPIDkD, turnprofile);
    RRturnPID = new ProfiledPIDController(turnPIDkP, 0, turnPIDkD, turnprofile);
    headingPID = new ProfiledPIDController(headingPIDkP, 0, headingPIDkD, headingprofile);

    //PID loop options
    FLturnPID.enableContinuousInput(-pi, pi);
    FRturnPID.enableContinuousInput(-pi, pi);
    RLturnPID.enableContinuousInput(-pi, pi);
    RRturnPID.enableContinuousInput(-pi, pi);
    headingPID.enableContinuousInput(-pi, pi);

    //linear filter instantiation
    FRturnfilter = LinearFilter.singlePoleIIR(0.2, 0.02);
    FLturnfilter = LinearFilter.singlePoleIIR(0.2, 0.02);
    RLturnfilter = LinearFilter.singlePoleIIR(0.2, 0.02);
    RRturnfilter = LinearFilter.singlePoleIIR(0.2, 0.02);
  }
  
  public void driveIKinVCL(double Xspeed, double Yspeed, double Zrot, double HeadingDesired, boolean HeadingCL) {
    //conditionals for heading controller
    headingdes = MathUtil.angleModulus(HeadingDesired);
    double Zinput;
    if (HeadingCL) {
      Zinput = Zrot;
    } else {
      headingPIDout = headingPID.calculate(headingactual, headingdes);
      Zinput = headingPIDout;

    }

    //Chassis Speeds object for IK calcs
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(Xspeed, Yspeed, Zinput, new Rotation2d(headingactual));
    ChassisSpeeds.discretize(speeds, 0.02);

    //IK calcs
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxspeedms);
    SwerveModuleState[] optimizedstates = new SwerveModuleState[] {
      SwerveModuleState.optimize(states[0], new Rotation2d(FLturnposactual)),
      SwerveModuleState.optimize(states[1], new Rotation2d(FRturnposactual)),
      SwerveModuleState.optimize(states[2], new Rotation2d(RLturnposactual)),
      SwerveModuleState.optimize(states[3], new Rotation2d(RRturnposactual))
    };

    //reduce speed by inverse of angle error
    optimizedstates[0].speedMetersPerSecond *= Math.cos(FLturnposactual - FLturnposdes);
    optimizedstates[1].speedMetersPerSecond *= Math.cos(FRturnposactual - FRturnposdes);
    optimizedstates[2].speedMetersPerSecond *= Math.cos(RLturnposactual - RLturnposdes);
    optimizedstates[3].speedMetersPerSecond *= Math.cos(RRturnposactual - RRturnposdes);

    //write internal generated states to vars
    FLturnposdes = optimizedstates[0].angle.getRadians();
    FRturnposdes = optimizedstates[1].angle.getRadians();
    RLturnposdes = optimizedstates[2].angle.getRadians();
    RRturnposdes = optimizedstates[3].angle.getRadians();
    FLdriveveldes = optimizedstates[0].speedMetersPerSecond;
    FRdriveveldes = optimizedstates[1].speedMetersPerSecond;
    RLdriveveldes = optimizedstates[2].speedMetersPerSecond;
    RRdriveveldes = optimizedstates[3].speedMetersPerSecond;

    //PID calculations
    FLturnPIDout = FLturnfilter.calculate(FLturnPID.calculate(FLturnposactual, FLturnposdes));
    FRturnPIDout = FRturnfilter.calculate(FRturnPID.calculate(FRturnposactual, FRturnposdes));
    RLturnPIDout = RLturnfilter.calculate(RLturnPID.calculate(RLturnposactual, RLturnposdes));
    RRturnPIDout = RRturnfilter.calculate(RRturnPID.calculate(RRturnposactual, RRturnposdes));

    //write PID calculations and speeds for drive
    FLdrive.setVoltage((FLdriveveldes / maxspeedms) * maxappliedvoltage);
    FRdrive.setVoltage((FRdriveveldes / maxspeedms) * maxappliedvoltage);
    RLdrive.setVoltage((RLdriveveldes / maxspeedms) * maxappliedvoltage);
    RRdrive.setVoltage((RRdriveveldes / maxspeedms) * maxappliedvoltage);
    FLturn.setVoltage((FLturnPIDout / maxrotspeedrads) * maxappliedvoltage);
    FRturn.setVoltage((FRturnPIDout / maxrotspeedrads) * maxappliedvoltage);
    RLturn.setVoltage((RLturnPIDout / maxrotspeedrads) * maxappliedvoltage);
    RRturn.setVoltage((RRturnPIDout / maxrotspeedrads) * maxappliedvoltage);
  }



  /**
   * 
   * @return Array of doubles with module data.
   * 0 = FLdrivevel
   * 1 = FLturnpos
   * 2 = FRdrivevel
   * 3 = FRturnpos
   * 4 = RLdrivevel
   * 5 = RLturnpos
   * 6 = RRdrivevel
   * 7 = RRturnpos
   */
  public double[] GetDrivetrainData(){
    return new double[] {
    FLdrivevelactual,
    FLturnposactual,
    FRdrivevelactual,
    FRturnposactual,
    RLdrivevelactual,
    RLturnposactual,
    RRdrivevelactual,
    RRturnposactual,
    headingactual
    };
  }

  public double[] GetDesiredData() {
    return new double[] {
      FLdriveveldes,
      FLturnposdes,
      FRdriveveldes,
      FRturnposdes,
      RLdriveveldes,
      RLturnposdes,
      RRdriveveldes,
      RRturnposdes,
      headingdes
    };
  }

  public double[] GetPIDoutData() {
    return new double[] {
      0,
      0,
      0,
      0,
      FLturnPIDout,
      FRturnPIDout,
      RLturnPIDout,
      RRturnPIDout,
      headingPIDout
    };
  }

  @Override
  public void periodic() {


    //gyroscope and encoder value condtioning, write conditioned values
    //+0.5pi rotation needed
    FLdrivevelactual = FLdriveencoder.getVelocity();
    double FLturnraw = FLturnencoder.getPosition() - 0.5*pi;
    FLturnposactual = -(FLturnraw - (pi*Math.signum(FLturnraw)));
    //+1pi rotation
    FRdrivevelactual = FRdriveencoder.getVelocity();
    double FRturnraw = FRturnencoder.getPosition();
    FRturnposactual = -(FRturnraw - (pi*Math.signum(FRturnraw)));
    //no zero offset needed
    RLdrivevelactual = RLdriveencoder.getVelocity();
    double RLturnraw = RLturnencoder.getPosition() - pi;
    RLturnposactual = -(RLturnraw - (pi*Math.signum(RLturnraw)));
    //-0.5pi rotation
    RRdrivevelactual = RRdriveencoder.getVelocity();
    double RRturnraw = RRturnencoder.getPosition() - 1.5*pi;
    RRturnposactual = -(RRturnraw - (pi*Math.signum(RRturnraw)));
    //gyro
    headingactual = MathUtil.angleModulus((-gyro.getAngle()/180)*pi);

  }
}
