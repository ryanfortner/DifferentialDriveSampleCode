// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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

  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    rightEncoder.setPositionConversionFactor(DriveConstants.kLinearDistanceConversionFactor);
    leftEncoder.setPositionConversionFactor(DriveConstants.kLinearDistanceConversionFactor);
    rightEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistanceConversionFactor / 60);
    leftEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistanceConversionFactor / 60);

    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);


    rightFrontMotor.setInverted(false);
    leftFrontMotor.setInverted(true);

    odometry = new DifferentialDriveOdometry(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    navX.reset();
    resetEncoders();
    odometry.resetPosition(navX.getRotation2d(), 0, 0, new Pose2d());
  }
  public void arcadeDrive(double fwd, double rot) {
    differentialDrive. arcadeDrive(fwd, rot);
  }

  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }


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

  public double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public double getAverageEncoderPosition() {
    return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
  }

  public void zeroHeading() {
    navX.reset();
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
    odometry.update(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    SmartDashboard.putNumber("left encoder value meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("right encoder value meters", getRightEncoderPosition());
    SmartDashboard.putNumber("gyro", getHeading());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
