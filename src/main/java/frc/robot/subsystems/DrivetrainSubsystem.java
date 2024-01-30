// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;  
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  CANSparkBase leftFrontMotor = new CANSparkMax(Constants.OperatorConstants.leftFrontMotorPort, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkBase leftBackMotor = new CANSparkMax(Constants.OperatorConstants.leftBackMotorPort, CANSparkLowLevel.MotorType.kBrushless); 
  CANSparkBase rightFrontMotor = new CANSparkMax(Constants.OperatorConstants.rightFrontMotorPort, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkBase rightBackMotor = new CANSparkMax(Constants.OperatorConstants.rightBackMotorPort, CANSparkLowLevel.MotorType.kBrushless);

  RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
  RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

  DifferentialDrive differentialDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
  AHRS navX = new AHRS(SPI.Port.kMXP);

  DifferentialDriveOdometry odometry;
  DifferentialDriveKinematics kinematics =
  new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);

  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    // Resets motor controllers (good practice)
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    // Converts encoder units (ticks) into meters
    rightEncoder.setPositionConversionFactor(DriveConstants.kLinearDistanceConversionFactor);
    leftEncoder.setPositionConversionFactor(DriveConstants.kLinearDistanceConversionFactor);
    rightEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistanceConversionFactor / 60);
    leftEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistanceConversionFactor / 60);
    
    // Makes back follow front so each side operates as 1
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    // Inversions due to motor setup
    rightFrontMotor.setInverted(false);
    leftFrontMotor.setInverted(true);

    odometry = new DifferentialDriveOdometry(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    navX.reset();
    resetEncoders();
    odometry.resetPosition(navX.getRotation2d(), 0, 0, new Pose2d());

    AutoBuilder.configureRamsete(
                this::getPose2d, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry 
                this::curChassisSpeeds, // Current ChassisSpeeds supplier
                this::drive, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. 
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE (VERY IMPORTANT)

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }
    // Uses forward speed (joystick input) and turning speed (joystick input)
  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }

  public ChassisSpeeds curChassisSpeeds() {
    // Gets wheel speeds in differentialDrive class using method 
    DifferentialDriveWheelSpeeds wheelSpeeds = getWheelSpeeds();
    // Converts differentialDrive wheel speeds into chassisSpeed wheels
    return kinematics.toChassisSpeeds(wheelSpeeds);
}

public void drive(ChassisSpeeds speeds) {
  DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

  // Hypothetical maximum speed in meters per second (unknown for now)
  double maxSpeedMetersPerSecond = 3.0; 

  // Convert from meters per second to -1 to 1 range by dividing cur by max
  double leftSpeed = (wheelSpeeds.leftMetersPerSecond / maxSpeedMetersPerSecond);
  double rightSpeed = (wheelSpeeds.rightMetersPerSecond / maxSpeedMetersPerSecond);

  // Ensure the values are within -1 to 1 range
  leftSpeed = Math.max(-1, Math.min(leftSpeed, 1)) * 0.2;
  rightSpeed = Math.max(-1, Math.min(rightSpeed, 1)) * -0.2;

  SmartDashboard.putNumber("leftSpeed", leftSpeed);
  SmartDashboard.putNumber("rightSpeed", rightSpeed);

  // Set the speeds to the motors
  leftFrontMotor.set(leftSpeed * -1);
  rightFrontMotor.set(rightSpeed);
  differentialDrive.feed()
  ;
}

  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  // Gets right and left encoder positions
  public double getRightEncoderPosition() {
    return -rightEncoder.getPosition();
  }
  public double getLeftEncoderPosition() {
    return -leftEncoder.getPosition();
  }

  public double getRightEncoderVelocity() {
    return rightEncoder.getVelocity();
  }
  public double getLeftEncoderVelocity() {
    return leftEncoder.getVelocity();
  }

  // Gets wheel speeds of each wheel
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  // feeds voltage into motors
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftFrontMotor.setVoltage(leftVolts);
    rightFrontMotor.setVoltage(rightVolts);
    differentialDrive.feed();
  } 

  public RelativeEncoder getLeftEncoder() {
    return leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  // gets degrees turned using gyro
  public double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  // Gets average "position" by taking average of both encoders
  public double getAverageEncoderPosition() {
    return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
  }

  public void zeroHeading() {
    navX.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    navX.reset();

    odometry.resetPosition(navX.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition(), getPose2d());
  }



  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // Updates odometry to make sure we have the right position/degrees all the time.
    odometry.update(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    // Simple printing
    SmartDashboard.putNumber("left encoder value meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("right encoder value meters", getRightEncoderPosition());
    SmartDashboard.putNumber("gyro", getHeading());
    differentialDrive.feed();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
