// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  CANSparkBase leftFrontMotor = new CANSparkMax(Constants.OperatorConstants.leftFrontMotorPort, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkBase leftBackMotor = new CANSparkMax(Constants.OperatorConstants.leftBackMotorPort, CANSparkLowLevel.MotorType.kBrushless); 
  CANSparkBase rightFrontMotor = new CANSparkMax(Constants.OperatorConstants.rightFrontMotorPort, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkBase rightBackMotor = new CANSparkMax(Constants.OperatorConstants.rightBackMotorPort, CANSparkLowLevel.MotorType.kBrushless);

  RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
  RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

  MotorControllerGroup leftControllerGroup = new MotorControllerGroup(leftBackMotor, leftFrontMotor);
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightBackMotor, rightFrontMotor);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);


  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    rightControllerGroup.setInverted(true);
    leftControllerGroup.setInverted(false);
  }

  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
