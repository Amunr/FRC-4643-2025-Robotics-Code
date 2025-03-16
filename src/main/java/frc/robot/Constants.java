// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class controlConstants
  {
    public static final byte driveController = 0;
    public static final byte operatorContoller = 1;
    public static final double deadband = 0.05;
    
  }
  public static class elevatorConstants {
    public static final int leftElevatorCAN = 14;
    public static final int rightElevatorCAN = 15;
    public static final int level1Rotations = 10;
    public static final int level2Rotations = 30;
    public static final int level3Rotations = 50;
    public static final int level4Rotations = 70;
    public static final int levelIntakeRotations = 0;
  }
  public static class coralConstants {
    public static final int leftCoralCAN = 16;
    public static final int rightCoralCAN = 17;
    public static final int backBeamBreakPort = 2;
    public static final int frontBeamBreakPort = 1; 
  }

  public static class climberConstants {
  public static final int climberMotorCAN = 18;
  }
}