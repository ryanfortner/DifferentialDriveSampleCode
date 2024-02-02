// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // Constant of each id for each CANSparkMAX Motor Controller
    public static final int leftBackMotorPort = 1;
    public static final int leftFrontMotorPort = 2;
    public static final int rightBackMotorPort = 3;
    public static final int rightFrontMotorPort = 4;
  }

  public static final class DriveConstants {
    public static final double kGearRatio = 10.71;
    public static final double kWheelRadiusInches = 3;

    public static final double kLinearDistancePerMotorRotation = (0.0254 * (1 / (kGearRatio * 2 * Math.PI *
        0.0254 * (kWheelRadiusInches)) * 10));

    // Efficiency values from SysID
    public static final double ksVolts = -0.045772;
    public static final double kvVoltSecondsPerMeter = 3.0212;
    public static final double kaVoltSecondsSquaredPerMeter = 0.42369;
    public static final double kPDriveVel = 3.839;

    // 21.75 inches equals 0.57785. trackwidth is horizontal distance between the
    // wheels
    public static final double kTrackwidthMeters = Units.inchesToMeters(22.875);
    // DifferentialDriveKinematics allows us to use the trackwidth to convert from
    // chassis speeds to wheel speeds. As elsewhere, we keep our units in meters.
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kLinearDistanceConversionFactor = (Units
        .inchesToMeters(1 / (kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) * 10));

    // public static final double kLinearDistanceConversionFactor = (0.0254 * (1 /
    // (kGearRatio * 2 * Math.PI * 0.0254 * (kWheelRadiusInches)) * 10));
  }

}
