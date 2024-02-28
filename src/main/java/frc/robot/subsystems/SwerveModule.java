// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;
  
  private PID rotorPID;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private CANcoder angleEncoder;

  private final SparkPIDController driveController;

  private final SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(Constants.Swerve.SwerveConstants.driveKS, Constants.Swerve.SwerveConstants.driveKV, Constants.Swerve.SwerveConstants.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID, Constants.Robot.canbus);

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
  
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.SwerveConstants.maxSpeed * 0.01)) 
      ? lastAngle
      : desiredState.angle;

    desiredState.angle = angle;

    double error = getState().angle.getDegrees() - desiredState.angle.getDegrees();
    double constrainedError = MathUtility.constrainAngleDegrees(error);
    double rotorOutput = rotorPID.calculate(constrainedError);
    rotorOutput = MathUtility.clamp(rotorOutput, -1, 1);
    angleMotor.set(rotorOutput);
    lastAngle = angle;

    if(isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.SwerveConstants.maxSpeed;
      percentOutput = MathUtility.clamp(percentOutput, -1, 1);
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
        desiredState.speedMetersPerSecond,
        ControlType.kVelocity,
        0,
        FeedForward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kVelocityOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.SwerveConstants.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.SwerveConstants.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.SwerveConstants.angleNeutralMode);
    rotorPID = new PID(Constants.Swerve.SwerveConstants.angleKP, 0, Constants.Swerve.SwerveConstants.angleKD, 0, 0);
    angleMotor.enableVoltageCompensation(Constants.Swerve.SwerveConstants.voltageComp);
  }
  
  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.SwerveConstants.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.SwerveConstants.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.SwerveConstants.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.SwerveConstants.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.SwerveConstants.driveConversionPositionFactor);
    driveController.setP(Constants.Swerve.SwerveConstants.driveKP);
    driveController.setI(Constants.Swerve.SwerveConstants.driveKI);
    driveController.setD(Constants.Swerve.SwerveConstants.driveKD);
    driveController.setFF(Constants.Swerve.SwerveConstants.driveKFF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.SwerveConstants.voltageComp);
    driveEncoder.setPosition(0.0);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue()).plus(angleOffset);
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
      driveEncoder.getPosition(), 
      getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public void setAngleMotor(double speed) {
    angleMotor.set(speed);
  }

  public void setDriveMotor(double speed) {
    driveMotor.set(speed);
  }
}
