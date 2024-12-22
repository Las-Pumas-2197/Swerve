// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**Used to store constants for the swerve drive.*/
public class swerveconfig {

    //mmmmm pi
    private static final double pi = Math.PI;

    /**Swerve module configuration data.*/
    public class moduleconfig {

        //max applied volts
        public static final double maxvolts = 12;

        //conversion factors
        public static final double con_azimuth_velfactor = 2*pi; //returns in rads/s
        public static final double con_azimuth_posfactor = 2*pi; //returns in rads
        public static final double con_drive_velfactor = (0.0762*pi / 4.71428) / 60; //returns in meters/s
        public static final double con_drive_posfactor = (0.0762*pi / 4.71428); //returns in meters
        
        //azimuth PID and FF gains, velocity constraints
        public static final double pid_azimuth_kP = 0.01; //needs tuned
        public static final double pid_azimuth_kI = 0; //try not to use
        public static final double pid_azimuth_kD = 0; //may be needed, high values will cause overshoot and hysteresis
        public static final double ff_azimuth_kS = 0.15; //measured
        public static final double ff_azimuth_kV = 0.94; //theoretical calculation, seems OK during testing in v5.3
        public static final double ff_azimuth_kA = 0; //0 = infinite, may be OK to omit in FF due to low module inertia
        public static final double azimuth_maxvel = 4*pi; //rads/s, theoretical maximum
        public static final double azimuth_maxacl = 8*pi; //rads/s^2, unable to measure so a high value is probably ok
        public static final TrapezoidProfile.Constraints azimuth_constraints = 
            new TrapezoidProfile.Constraints(azimuth_maxvel, azimuth_maxacl); //send it
        
        //drive FF gains
        public static final double ff_drive_kS = 0.1; //needs characterized, likely extremely low
        public static final double ff_drive_kV = 2.09; //needs checked, calculated theoretical 
        public static final double ff_drive_kA = 0.24; //needs checked, calculated theoretical
        public static final double drive_maxvel = 5.7; //m/s, calculated maximum
    }

    /**Main swerve drive configuration data.*/
    public class mainconfig {
        
        //module IDs
        public static final int FLazimuthID = 3;
        public static final int FRazimuthID = 5;
        public static final int RLazimuthID = 9;
        public static final int RRazimuthID = 7;
        public static final int FLdriveID = 4;
        public static final int FRdriveID = 6;
        public static final int RLdriveID = 10;
        public static final int RRdriveID = 8;
        public static final int gyroID = 20;

        //drivetrain dimensions for kinematics
        public static final double trackwidth = Units.inchesToMeters(26.5);
        public static final double wheelbase = Units.inchesToMeters(26.5);

        //module offsets
        public static final double FLoffset = -0.5*pi;
        public static final double FRoffset = 0*pi;
        public static final double RLoffset = -1*pi;
        public static final double RRoffset = -1.5*pi;

        //heading constraints and profile
        public static final double pid_headingkP = 0.01;
        public static final double pid_headingkI = 0;
        public static final double pid_headingkD = 0;
        public static final double ff_headingkS = 0; //need measured
        public static final double ff_headingkV = 5.45; //calculated theoretical
        public static final double ff_headingkA = 0.1;
        public static final double heading_maxvel = 2*pi; //need calculated
        public static final double heading_maxacl = 1*pi; //need calculated
        public static final TrapezoidProfile.Constraints heading_constraints=
            new TrapezoidProfile.Constraints(heading_maxvel, heading_maxacl);

        //max velocity and acceleration constraints for drivetrain, vel in m/s, acl in m/s^2
        public static final double maxrotvel_teleop = 2*pi; //same as heading_maxvel for now
        public static final double maxlinvel_teleop = 3.0; //60% is good due to motor power curves and headspace for rotation
        public static final double maxlinvel_auto = 2.0; //low and slow, speed up if needed
        public static final double maxlinvel_turbo = 5.7; //spicy
        public static final double maxlinacl = 3.0; //needs measured/calculated

        //kinematics for drivetrain
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(wheelbase/2, trackwidth/2), //FL
            new Translation2d(wheelbase/2, -trackwidth/2), //FR
            new Translation2d(-wheelbase/2, trackwidth/2), //RL
            new Translation2d(-wheelbase/2, -trackwidth/2)); //RR
    }
}
