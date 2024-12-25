// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Translation2d;

/**Used to store constants for the swerve drive.*/
public class swerveconfig {

    //mmmmm pi
    private static final double pi = Math.PI;
    private static final double rad = 2*pi;

    /**Swerve module configuration data.*/
    public class moduleconfig {

        //max applied volts
        public static final double maxvolts = 12;

        //conversion factors
        public static final double con_azimuth_velfactor = rad; //returns in rads/s, natively in rot/sec
        public static final double con_azimuth_posfactor = rad; //returns in rads
        public static final double drive_finalratio = 4.71428; //final drive ratio
        public static final double con_drive_velfactor = (0.0762*pi / drive_finalratio) / 60; //returns in meters/s
        public static final double con_drive_posfactor = (0.0762*pi / drive_finalratio); //returns in meters
        
        //azimuth PID and FF gains, velocity constraints
        public static final double pid_azimuth_kP = 0.01; //needs tuned
        public static final double pid_azimuth_kD = 0; //may be needed
        public static final double ff_azimuth_kS = 0; //need measured
        public static final double ff_azimuth_kV = 0.4847; //calculated
        public static final double ff_azimuth_kA = 0.6024; //calculated at 0.6024 linear
        public static final double azimuth_maxvel = 3.94*rad; //rads/s
        public static final double azimuth_maxacl = 3.17*rad; //rads/s^2, calculated off of 5lbs and stall torque
        public static final TrapezoidProfile.Constraints azimuth_constraints = 
            new TrapezoidProfile.Constraints(azimuth_maxvel, azimuth_maxacl); //send it
        
        //drive FF gains
        public static final double ff_drive_kS = 0.1; //need measured
        public static final double ff_drive_kV = 2.09; //calculated
        public static final double ff_drive_kA = 0.23; //calculated @60lb weight, 0.45 @ 150lb dressed weight
        public static final double drive_maxvel = 5.74; //m/s, calculated maximum @12v
        public static final double drive_maxacl = 10.46; //m/s^2, absolute max calculated before wheel slip
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
        public static final double trackwidth = 0.6731; //meters, 26.5 inches
        public static final double wheelbase = 0.6731; //meters, 26.5 inches

        //module offsets
        public static final double FLoffset = -0.5*pi;
        public static final double FRoffset = 0*pi;
        public static final double RLoffset = 1*pi;
        public static final double RRoffset = 1.5*pi;

        //calculations for max heaing rot velocity

        //heading/rot constraints and profile
        public static final double pid_headingkP = 0.01;
        public static final double pid_headingkD = 0;
        public static final double ff_headingkS = 0; //need measured
        public static final double ff_headingkV = 0.50; //calculated
        public static final double ff_headingkA = 0.54; //calculated 0.5419 linear
        public static final double heading_maxvel = 3.84*rad; //calculated
        public static final double heading_maxacl = 3.52*rad; //calculated @60lb weight with stall torque, 1.4*rad @150lbs
        public static final double heading_maxvel_lim = 2*rad; //limited for better control
        public static final double heading_maxacl_lim = 1*rad; //same
        public static final TrapezoidProfile.Constraints heading_constraints=
            new TrapezoidProfile.Constraints(heading_maxvel_lim, heading_maxacl_lim);

        //max velocity and acceleration constraints for drivetrain, vel in m/s, acl in m/s^2        
        public static final double maxlinvel = 5.74; //max theoretical
        public static final double maxlinacl = 10.46; //same
        public static final double maxlinvel_teleop = 3.0; //adjust as needed
        public static final double maxlinacl_teleop = 3.0; //same
        public static final double maxlinvel_auto = 3.0; //adjust as needed
        public static final double maxlinacl_auto = 3.0; //same

        //kinematics for drivetrain
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(wheelbase/2, trackwidth/2), //FL
            new Translation2d(wheelbase/2, -trackwidth/2), //FR
            new Translation2d(-wheelbase/2, trackwidth/2), //RL
            new Translation2d(-wheelbase/2, -trackwidth/2)); //RR
    }
}
