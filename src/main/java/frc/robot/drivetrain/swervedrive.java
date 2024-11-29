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

import edu.wpi.first.math.controller.ArmFeedforward;
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
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
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
  private final SimpleMotorFeedforward FLturnFF;
  private final SimpleMotorFeedforward FRturnFF;
  private final SimpleMotorFeedforward RLturnFF;
  private final SimpleMotorFeedforward RRturnFF;
  private final SimpleMotorFeedforward headingFF;

  //linear filters for PID outputs
  private final LinearFilter FLturnfilter;
  private final LinearFilter FRturnfilter;
  private final LinearFilter RLturnfilter;
  private final LinearFilter RRturnfilter;

  //module PID output values for diagnostics
  private double FLturnPIDout;
  private double FRturnPIDout;
  private double RLturnPIDout;
  private double RRturnPIDout;

  //heading PID output value
  private double headingPIDout;

  //FF output values for diagnostics
  private double FLdriveFFout;
  private double FRdriveFFout;
  private double RLdriveFFout;
  private double RRdriveFFout;
  private double FLturnFFout;
  private double FRturnFFout;
  private double RLturnFFout;
  private double RRturnFFout;

  //heading FF output
  private double headingFFout;

  //angle error values
  private double FLturnerror;
  private double FRturnerror;
  private double RLturnerror;
  private double RRturnerror;

  //heading angle error value
  private double headingerror;

  //trapezoidal constraints for turn PID
  private final TrapezoidProfile.Constraints turnprofile;
  private final TrapezoidProfile.Constraints headingprofile;

  //tuning parameters for PID loops and FFs, FFs not implemented yet
  private static final double turnPIDkP = 0.01; //previously 1.25 in v5.2, tune again
  private static final double turnPIDkD = 0; //previously 0.35 in v5.2, tune again
  private static final double turnFFkS = 0.1; //needs measured!!!!! in V
  private static final double turnFFkV = 0.94; //needs measured!!!! in V*rads/s
  private static final double turnFFkA = 0; //needs calculated, V*rads/s^2, zero is probably ok due to low inertia
  private static final double turntol = 0.05*pi;

  //tuning parameters for drive loops and FFs, gains calculated with 50lb robot
  private static final double driveFFkS = 0.1; //needs measured!!! in V
  private static final double driveFFkV = 2.09; //needs measured!!! in V*m/s
  private static final double driveFFkA = 0.24; //needs measured!!! in V*m/s^2

  //tuning parameters for heading, needs adjusted for radians
  private static final double headingPIDkP = 0.01; //previously 6
  private static final double headingPIDkD = 0; //previously 0
  private static final double headingFFkS = 0.1; //needs measured!!!!! in V
  private static final double headingFFkV = 5.45; //needs measured!!!!! in V*rads/s
  private static final double headingFFkA = 0; //needs calculated!!!! in V*rads/s^2
  private static final double headingtol = 0.05*pi;

  //drive constraints for motion profile
  private static final double maxmodulevelmps = 5.7; //needs measured!!!!! calculated at 5.7 m/s
  private static final double maxmoduleaclmps = 2.5; //needs measured!!!!!, units m/s^2

  //max limits for turn and constraints, constraints must be equal or less than maximum
  private static final double maxmodulevelrads = 4*pi; //measured, units rads/s
  private static final double maxmoduleaclrads = 8*pi; //needs measured, units rads/s^2
  private static final double turnvelconstraint = 4*pi;
  private static final double turnaclconstraint = 2*pi;

  //heading limits and constraints for motion profile
  private static final double maxheadingvelrads = 2.11*pi; //needs measured!!!!!
  private static final double maxheadingaclrads = 1*pi; //needs measured!!!!!
  private static final double headingvelconstraint = 2*pi;
  private static final double headingaclconstraint = 1*pi;

  //voltage constraints
  private static final double maxappliedvoltage = 12;

  //min and max for pid loops
  private static final double turnPIDoutmin = 0.1;
  private static final double turnPIDoutmax = 10;
  private static final double headingPIDoutmin = 0.2;
  private static final double headingPIDoutmax = 10;

  //conversion factors for drive
  private static final double driveposconvfactor = (0.0762*pi / 4.71428); //meters  
  private static final double drivevelconvfactor = (0.0762*pi / 4.71428) / 60; //m/s

  //conversion factors for turn
  private static final double turnposconvfactor = 2*pi; //rads
  private static final double turnvelconvfactor = 2*pi; //rads/s, div by 60 unneeded, encoder in RPS units

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
    headingprofile = new TrapezoidProfile.Constraints(headingvelconstraint, headingaclconstraint);
    turnprofile = new TrapezoidProfile.Constraints(turnvelconstraint, turnaclconstraint); //rads/s and rads/s^2

    //instantiate PID loops
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
    FLturnFF = new SimpleMotorFeedforward(turnFFkS, turnFFkV, turnFFkA);
    FRturnFF = new SimpleMotorFeedforward(turnFFkS, turnFFkV, turnFFkA);
    RLturnFF = new SimpleMotorFeedforward(turnFFkS, turnFFkV, turnFFkA);
    RRturnFF = new SimpleMotorFeedforward(turnFFkS, turnFFkV, turnFFkA);
    FLdriveFF = new SimpleMotorFeedforward(driveFFkS, driveFFkV, driveFFkA);
    FRdriveFF = new SimpleMotorFeedforward(driveFFkS, driveFFkV, driveFFkA);
    RLdriveFF = new SimpleMotorFeedforward(driveFFkS, driveFFkV, driveFFkA);
    RRdriveFF = new SimpleMotorFeedforward(driveFFkS, driveFFkV, driveFFkA);
    headingFF = new SimpleMotorFeedforward(headingFFkS, headingFFkV, headingFFkA);

    //linear filter instantiation, may or may not be needed for later versions
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
  public void drive(double Xspeed, double Yspeed, double Zrot, double HeadingDesired, boolean HeadingCL) {

    //conditionals for heading controller
    headingdes = MathUtil.angleModulus(HeadingDesired);

    double Zinput;
    
    if (HeadingCL) {
      if (headingerror < headingtol) {
        headingPIDout = 0;
      } else {
        double headingPIDoutraw = headingPID.calculate(headingactual, headingdes);
        headingPIDout = 
          Math.signum(headingPIDoutraw)*MathUtil.clamp(
            Math.abs(headingPIDoutraw), headingPIDoutmin, headingPIDoutmax
          );
      }
      Zinput = headingPIDout;
    } else {
      Zinput = Zrot;
    }

    //Chassis Speeds object for IK calcs
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      Xspeed, Yspeed, Zinput, new Rotation2d(headingactual)
    );
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

    //write internal generated states to vars and reduce drive vel by inverse of angle error
    //may average angle errors for total error and multiply velocity by inverse for less hysteresis
    FLturnposdes = optimizedstates[0].angle.getRadians();
    FRturnposdes = optimizedstates[1].angle.getRadians();
    RLturnposdes = optimizedstates[2].angle.getRadians();
    RRturnposdes = optimizedstates[3].angle.getRadians();
    FLdriveveldes = optimizedstates[0].speedMetersPerSecond * Math.cos(FLturnerror);
    FRdriveveldes = optimizedstates[1].speedMetersPerSecond * Math.cos(FRturnerror);
    RLdriveveldes = optimizedstates[2].speedMetersPerSecond * Math.cos(RLturnerror);
    RRdriveveldes = optimizedstates[3].speedMetersPerSecond * Math.cos(RRturnerror);

    //PID calculations
    FLturnPIDout = FLturnfilter.calculate(FLturnPID.calculate(FLturnposactual, FLturnposdes));
    FRturnPIDout = FRturnfilter.calculate(FRturnPID.calculate(FRturnposactual, FRturnposdes));
    RLturnPIDout = RLturnfilter.calculate(RLturnPID.calculate(RLturnposactual, RLturnposdes));
    RRturnPIDout = RRturnfilter.calculate(RRturnPID.calculate(RRturnposactual, RRturnposdes));
    
    //drive FF calculations, need to check output units and convert as necessary
    FLturnFFout = FLturnFF.calculate(FLturnposactual, FLturnposdes, 0.02);
    FRturnFFout = FRturnFF.calculate(FRturnposactual, FLturnposdes, 0.02);
    RLturnFFout = RLturnFF.calculate(RLturnposactual, FLturnposdes, 0.02);
    RRturnFFout = RRturnFF.calculate(RRturnposactual, FLturnposdes, 0.02);
    FLdriveFFout = FLdriveFF.calculate(FLdrivevelactual, FLdriveveldes, 0.02);
    FRdriveFFout = FRdriveFF.calculate(FRdrivevelactual, FRdriveveldes, 0.02);
    RLdriveFFout = RLdriveFF.calculate(RLdrivevelactual, RLdriveveldes, 0.02);
    RRdriveFFout = RRdriveFF.calculate(RRdrivevelactual, RRdriveveldes, 0.02);

    //write PID calculations and speeds for drive
    FLdrive.setVoltage(0);
    FRdrive.setVoltage(0);
    RLdrive.setVoltage(0);
    RRdrive.setVoltage(0);
    FLturn.setVoltage(0);
    FRturn.setVoltage(0);
    RLturn.setVoltage(0);
    RRturn.setVoltage(0);
  }

  /** Telemetry manager in swervemain.
   * Write true for which values are desired to be written and
   * monitored.
   * @param ModulePosValues Writes module positional values.
   * @param ModuleVelValues Writes module velocity values.
   * @param ModulePIDValues Writes PID controller outputs.
   * @param ModuleFFValues Writes FF controller outputs.
   * @param ModuleErrorValues Writes module and heading postional error values.
   * @param HeadingValues Writes heading values.
   * @param SpeedValues Writes input speed values.
   */
  public void telemetry(
    boolean ModulePosValues,
    boolean ModuleVelValues,
    boolean ModulePIDValues,
    boolean ModuleFFValues,
    boolean ModulePIDFValues,
    boolean ModuleErrorValues,
    boolean HeadingValues,
    boolean SpeedValues
  ) {

    if (ModulePosValues) {
      SmartDashboard.putNumber("FLTurnPosActual", FLturnposactual);
      SmartDashboard.putNumber("FRTurnPosActual", FRturnposactual);
      SmartDashboard.putNumber("RLTurnPosActual", RLturnposactual);
      SmartDashboard.putNumber("RRTurnPosActual", RRturnposactual);    
      SmartDashboard.putNumber("FLTurnPosDesired", FLturnposdes);
      SmartDashboard.putNumber("FRTurnPosDesired", FRturnposdes);
      SmartDashboard.putNumber("RLTurnPosDesired", RLturnposdes);
      SmartDashboard.putNumber("RRTurnPosDesired", RRturnposdes);
    }

    if (ModuleVelValues) {
      SmartDashboard.putNumber("FLDriveVelActual", FLdrivevelactual);
      SmartDashboard.putNumber("FRDriveVelActual", FRdrivevelactual);
      SmartDashboard.putNumber("RLDriveVelActual", RLdrivevelactual);
      SmartDashboard.putNumber("RRDriveVelActual", RRdrivevelactual);    
      SmartDashboard.putNumber("FLDriveVelDesired", FLdriveveldes);
      SmartDashboard.putNumber("FRDriveVelDesired", FRdriveveldes);
      SmartDashboard.putNumber("RLDriveVelDesired", RLdriveveldes);
      SmartDashboard.putNumber("RRDriveVelDesired", RRdriveveldes);
    }

    if (ModulePIDValues) {
      SmartDashboard.putNumber("FLturnPIDout", FLturnPIDout);
      SmartDashboard.putNumber("FRturnPIDout", FRturnPIDout);
      SmartDashboard.putNumber("RLturnPIDout", RLturnPIDout);
      SmartDashboard.putNumber("RRturnPIDout", RRturnPIDout);
    }

    if (ModuleFFValues) {
      SmartDashboard.putNumber("FLturnFFout", FLturnFFout);
      SmartDashboard.putNumber("FRturnFFout", FRturnFFout);
      SmartDashboard.putNumber("RLturnFFout", RLturnFFout);
      SmartDashboard.putNumber("RRturnFFout", RRturnFFout);    
      SmartDashboard.putNumber("FLdriveFFout", FLdriveFFout);
      SmartDashboard.putNumber("FRdriveFFout", FRdriveFFout);
      SmartDashboard.putNumber("RLdriveFFout", RLdriveFFout);
      SmartDashboard.putNumber("RRdriveFFout", RRdriveFFout);
    }
    
    if (ModulePIDFValues) {
            SmartDashboard.putNumber("FLturnPIDFFout", FLturnPIDout + FLturnFFout);
    }

    if (ModuleErrorValues) {
      SmartDashboard.putNumber("FLturnerror", FLturnerror);
      SmartDashboard.putNumber("FRturnerror", FRturnerror);
      SmartDashboard.putNumber("RLturnerror", RLturnerror);
      SmartDashboard.putNumber("RRturnerror", RRturnerror);
    }

    if (HeadingValues) {
      SmartDashboard.putNumber("HeadingActual", headingactual);
      SmartDashboard.putNumber("HeadingDesired", headingdes);
      SmartDashboard.putNumber("HeadingFFout", headingFFout);
      SmartDashboard.putNumber("HeadingPIDout", headingPIDout);
      SmartDashboard.putNumber("HeadingError", headingerror);
    }

    if (SpeedValues) {
      SmartDashboard.putNumber("Yspeeddes", Yspeeddes);
      SmartDashboard.putNumber("Zrotspeeddes", Zrotdes);
      SmartDashboard.putNumber("Xspeeddes", Xspeeddes);
    }








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
