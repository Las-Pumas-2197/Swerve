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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  //feedforward construction
  private final SimpleMotorFeedforward FLdriveFF;
  private final SimpleMotorFeedforward FRdriveFF;
  private final SimpleMotorFeedforward RLdriveFF;
  private final SimpleMotorFeedforward RRdriveFF;

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

  //FFoutput values for diagnostics
  private double FLdriveFFout;
  private double FRdriveFFout;
  private double RLdriveFFout;
  private double RRdriveFFout;

  //angle error values
  private double FLturnerror;
  private double FRturnerror;
  private double RLturnerror;
  private double RRturnerror;
  private double headingerror;

  //trapezoidal constraints for turn PID
  private final TrapezoidProfile.Constraints turnprofile;
  private final TrapezoidProfile.Constraints headingprofile;

  //misc tuning parameters for PID loops, FFs, and other calcs.
  //FF kV and kA are calculated by ReCalc, kS needs measured. Calced with 50lb robot weight 
  private static final double turnPIDkP = 1.25;
  private static final double turnPIDkD = 0.35;
  private static final double turntol = 0.05*pi;
  private static final double headingPIDkP = 6;
  private static final double headingPIDkD = 0;
  private static final double headingtol = 0.05*pi;
  private static final double driveFFkS = 0.12; //measured, used to model static friction of system
  private static final double driveFFkV = 2.09; //calculated by ReCalc, V*s/m
  private static final double driveFFkA = 0.47; //calculated by ReCalc, units of V*s^2/m

  //voltage constraints
  private static final double maxappliedvoltage = 12.6;

  //drive constraints for motion profile
  private static final double maxmodulevelmps = 5;

  //turn constraints for motion profile
  private static final double maxmodulevelrads = 4*pi; //measured
  private static final double maxmoduleaclrads = 8*pi; //needs measured

  //heading constraints for motion profile
  private static final double maxheadingvelrads = 2*pi;
  private static final double maxheadingaclrads = 2*pi;

  //min and max for pid loops
  private static final double turnPIDoutmin = 0.1;
  private static final double turnPIDoutmax = 10;
  private static final double headingPIDoutmin = 0.2;
  private static final double headingPIDoutmax = 10;

  //conversion factors for velocity
  private static final double drivevelconvfactor = (0.0762*pi / 4.71428) / 60; //m/s
  private static final double turnvelconvfactor = 2*pi; //rads/s

  //conversion factors for position 
  private static final double driveposconvfactor = (0.0762*pi / 4.71428); //meters
  private static final double turnposconvfactor = 2*pi; //rads

  //Initializations for desired speeds
  private double Yspeeddes;
  private double Xspeeddes;
  private double Zrotdes;

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
    headingprofile = new TrapezoidProfile.Constraints(maxheadingvelrads, maxheadingaclrads);
    turnprofile = new TrapezoidProfile.Constraints(maxmodulevelrads, maxmoduleaclrads); //rads/s and rads/s^2

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

    //instantiate feed forwards
    FLdriveFF = new SimpleMotorFeedforward(driveFFkS, driveFFkV, driveFFkA);
    FRdriveFF = new SimpleMotorFeedforward(driveFFkS, driveFFkV, driveFFkA);
    RLdriveFF = new SimpleMotorFeedforward(driveFFkS, driveFFkV, driveFFkA);
    RRdriveFF = new SimpleMotorFeedforward(driveFFkS, driveFFkV, driveFFkA);

    //linear filter instantiation
    FRturnfilter = LinearFilter.singlePoleIIR(0.2, 0.02);
    FLturnfilter = LinearFilter.singlePoleIIR(0.2, 0.02);
    RLturnfilter = LinearFilter.singlePoleIIR(0.2, 0.02);
    RRturnfilter = LinearFilter.singlePoleIIR(0.2, 0.02);
  }
  
  /** Drive robot with generated states from inverse kinematics in closed loop azimuth control
   * 
   * @param Xspeed
   * @param Yspeed
   * @param Zrot
   * @param HeadingDesired
   * @param HeadingCL
   */
  public void driveIKinVCL(double Xspeed, double Yspeed, double Zrot, double HeadingDesired, boolean HeadingCL) {

    //conditionals for heading controller
    headingdes = MathUtil.angleModulus(HeadingDesired);

    double Zinput;
    
    if (HeadingCL) {
      //if (headingerror < headingtol) {
      //  headingPIDout = 0;
      //} else {
      //  double headingPIDoutraw = headingPID.calculate(headingactual, headingdes);
      //  headingPIDout = Math.signum(headingPIDoutraw)*MathUtil.clamp(Math.abs(headingPIDoutraw), headingPIDoutmin, headingPIDoutmax);
      //}
      Zinput = headingPIDout;
    } else {
      Zinput = Zrot;
    }

    //Chassis Speeds object for IK calcs
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(Xspeed, Yspeed, Zinput, new Rotation2d(headingactual));
    ChassisSpeeds.discretize(speeds, 0.02);

    //IK calcs
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxmodulevelmps);
    SwerveModuleState[] optimizedstates = new SwerveModuleState[] {
      SwerveModuleState.optimize(states[0], new Rotation2d(FLturnposactual)),
      SwerveModuleState.optimize(states[1], new Rotation2d(FRturnposactual)),
      SwerveModuleState.optimize(states[2], new Rotation2d(RLturnposactual)),
      SwerveModuleState.optimize(states[3], new Rotation2d(RRturnposactual))
    };

    //reduce speed by inverse of angle error and add inflate error by multiplier to make more aggressive
    optimizedstates[0].speedMetersPerSecond *= Math.cos(FLturnerror);
    optimizedstates[1].speedMetersPerSecond *= Math.cos(FRturnerror);
    optimizedstates[2].speedMetersPerSecond *= Math.cos(RLturnerror);
    optimizedstates[3].speedMetersPerSecond *= Math.cos(RRturnerror);

    //write internal generated states to vars
    FLturnposdes = optimizedstates[0].angle.getRadians();
    FRturnposdes = optimizedstates[1].angle.getRadians();
    RLturnposdes = optimizedstates[2].angle.getRadians();
    RRturnposdes = optimizedstates[3].angle.getRadians();
    FLdriveveldes = optimizedstates[0].speedMetersPerSecond;
    FRdriveveldes = optimizedstates[1].speedMetersPerSecond;
    RLdriveveldes = optimizedstates[2].speedMetersPerSecond;
    RRdriveveldes = optimizedstates[3].speedMetersPerSecond;

    //PID calculations, commented out minimums to implement feedforwards iteratively
    
    
    //if (FLturnerror < turntol) {
    //  FLturnPIDout = 0;
    //} else {
    //  double FLturnPIDoutraw = FLturnfilter.calculate(FLturnPID.calculate(FLturnposactual, FLturnposdes));
    //  FLturnPIDout = Math.signum(FLturnPIDoutraw)*MathUtil.clamp(Math.abs(FLturnPIDoutraw), turnPIDoutmin, turnPIDoutmax);
    //}

    //if (FRturnerror < turntol) {
    //  FRturnPIDout = 0;
    //} else {
    //  double FRturnPIDoutraw = FRturnfilter.calculate(FRturnPID.calculate(FRturnposactual, FRturnposdes));
    //  FRturnPIDout = Math.signum(FRturnPIDoutraw)*MathUtil.clamp(Math.abs(FRturnPIDoutraw), turnPIDoutmin, turnPIDoutmax);
    //}
    
    //if (RLturnerror < turntol) {
    //  RLturnPIDout = 0;
    //} else {
    //  double RLturnPIDoutraw = RLturnfilter.calculate(RLturnPID.calculate(RLturnposactual, RLturnposdes));
    //  RLturnPIDout = Math.signum(RLturnPIDoutraw)*MathUtil.clamp(Math.abs(RLturnPIDoutraw), turnPIDoutmin, turnPIDoutmax);
    //}

    //if (RRturnerror < turntol) {
    //  RRturnPIDout = 0;
    //} else {
    //  double RRturnPIDoutraw = RRturnfilter.calculate(RRturnPID.calculate(RRturnposactual, RRturnposdes));
    //  RRturnPIDout = Math.signum(RRturnPIDoutraw)*MathUtil.clamp(Math.abs(RRturnPIDoutraw), turnPIDoutmin, turnPIDoutmax);
    //}
    
    //write PID calculations and speeds for drive
    FLdrive.setVoltage((FLdriveveldes / maxmodulevelmps) * maxappliedvoltage);
    FRdrive.setVoltage((FRdriveveldes / maxmodulevelmps) * maxappliedvoltage);
    RLdrive.setVoltage((RLdriveveldes / maxmodulevelmps) * maxappliedvoltage);
    RRdrive.setVoltage((RRdriveveldes / maxmodulevelmps) * maxappliedvoltage);
    FLturn.setVoltage((FLturnPIDout / maxmodulevelrads) * maxappliedvoltage);
    FRturn.setVoltage((FRturnPIDout / maxmodulevelrads) * maxappliedvoltage);
    RLturn.setVoltage((RLturnPIDout / maxmodulevelrads) * maxappliedvoltage);
    RRturn.setVoltage((RRturnPIDout / maxmodulevelrads) * maxappliedvoltage);
  }

  public void telemetry() {
    SmartDashboard.putNumber("FLTurnPosActual", FLturnposactual);
    SmartDashboard.putNumber("FRTurnPosActual", FRturnposactual);
    SmartDashboard.putNumber("RLTurnPosActual", RLturnposactual);
    SmartDashboard.putNumber("RRTurnPosActual", RRturnposactual);

    SmartDashboard.putNumber("FLTurnPosDesired", FLturnposdes);
    SmartDashboard.putNumber("FRTurnPosDesired", FRturnposdes);
    SmartDashboard.putNumber("RLTurnPosDesired", RLturnposdes);
    SmartDashboard.putNumber("RRTurnPosDesired", RRturnposdes);

    SmartDashboard.putNumber("FLDriveVelActual", FLdrivevelactual);
    SmartDashboard.putNumber("FRDriveVelActual", FRdrivevelactual);
    SmartDashboard.putNumber("RLDriveVelActual", RLdrivevelactual);
    SmartDashboard.putNumber("RRDriveVelActual", RRdrivevelactual);   
    
    SmartDashboard.putNumber("FLDriveVelDesired", FLdriveveldes);
    SmartDashboard.putNumber("FRDriveVelDesired", FRdriveveldes);
    SmartDashboard.putNumber("RLDriveVelDesired", RLdriveveldes);
    SmartDashboard.putNumber("RRDriveVelDesired", RRdriveveldes);

    SmartDashboard.putNumber("FLturnPIDout", FLturnPIDout);
    SmartDashboard.putNumber("FRturnPIDout", FRturnPIDout);
    SmartDashboard.putNumber("RLturnPIDout", RLturnPIDout);
    SmartDashboard.putNumber("RRturnPIDout", RRturnPIDout);

    SmartDashboard.putNumber("FLdriveFFout", 0);
    SmartDashboard.putNumber("FRdriveFFout", 0);
    SmartDashboard.putNumber("RLdriveFFout", 0);
    SmartDashboard.putNumber("RRdriveFFout", 0);

    SmartDashboard.putNumber("HeadingActual", headingactual);
    SmartDashboard.putNumber("HeadingDesired", headingdes);

    SmartDashboard.putNumber("HeadingDesired", headingdes);
    SmartDashboard.putNumber("Yspeeddes", Yspeeddes);
    SmartDashboard.putNumber("Zrotspeeddes", Zrotdes);
    SmartDashboard.putNumber("Xspeeddes", Xspeeddes);
    SmartDashboard.putNumber("HeadingPIDout", headingPIDout);

    SmartDashboard.putNumber("FLturnerror", FLturnerror);
    SmartDashboard.putNumber("FRturnerror", FRturnerror);
    SmartDashboard.putNumber("RLturnerror", RLturnerror);
    SmartDashboard.putNumber("RRturnerror", RRturnerror);
    SmartDashboard.putNumber("Headingerror", headingerror);
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
    //angle error calcs
    FLturnerror = Math.abs(FLturnposactual - FLturnposdes);
    FRturnerror = Math.abs(FRturnposactual - FRturnposdes);
    RLturnerror = Math.abs(RLturnposactual - RLturnposdes);
    RRturnerror = Math.abs(RRturnposactual - RRturnposdes);
    headingerror = Math.abs(headingactual - headingdes);

  }
}
