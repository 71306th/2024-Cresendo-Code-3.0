// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.robot.Constants;
import frc.robot.Constants.Upper.UpperConstants.UpperState;

public class Upper extends SubsystemBase {

  private final TalonFX tilterLeft;
  private final TalonFX tilterRight;
  private final CANSparkMax intakeLowerLeft;
  private final CANSparkMax intakeLowerRight;
  private final TalonSRX intakeUpper;
  
  private final CANcoder tilterEncoder;
  private final RelativeEncoder shooterLeftEncoder;
  private final RelativeEncoder shooterRightEncoder;
  
  private final I2C.Port i2cPort;
  private final ColorSensorV3 colorSensor;

  private final DigitalInput tilterLimitSwitch;

  private final PID tilterPID;
  private final PID lockPID;

  private UpperState state;

  private double tilterOutput;
  private double intakeOutput;

  public Upper() {
    /* declarings */
    tilterLeft = new TalonFX(Constants.Upper.UpperConstants.tilterLeft, Constants.Robot.canbus);
    tilterRight = new TalonFX(Constants.Upper.UpperConstants.tilterRight, Constants.Robot.canbus);
    
    intakeLowerLeft = new CANSparkMax(Constants.Upper.UpperConstants.intakeLowerLeft, MotorType.kBrushless);
    intakeLowerRight = new CANSparkMax(Constants.Upper.UpperConstants.intakeLowerRight, MotorType.kBrushless);
    intakeUpper = new TalonSRX(Constants.Upper.UpperConstants.intakeUpper);

    tilterEncoder = new CANcoder(Constants.Upper.UpperConstants.tilterEncoder, Constants.Robot.canbus);
    shooterLeftEncoder = intakeLowerLeft.getEncoder();
    shooterRightEncoder = intakeLowerRight.getEncoder();

    i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);

    tilterLimitSwitch = new DigitalInput(Constants.Upper.UpperConstants.tilterLimitSwitch);

    lockPID = new PID(
      Constants.Upper.UpperConstants.lockKP, 
      Constants.Upper.UpperConstants.lockKI, 
      Constants.Upper.UpperConstants.lockKD, 
      Constants.Upper.UpperConstants.lockiWindup, 
      Constants.Upper.UpperConstants.lockiLimit
    );
    tilterPID = new PID(
      Constants.Upper.UpperConstants.tilterKP, 
      Constants.Upper.UpperConstants.tilterKI, 
      Constants.Upper.UpperConstants.tilterKD, 
      Constants.Upper.UpperConstants.tilteriWindup, 
      Constants.Upper.UpperConstants.tilteriLimit
      );

    /* settings */
    intakeLowerLeft.setIdleMode(IdleMode.kCoast);
    intakeLowerRight.setIdleMode(IdleMode.kCoast);
    
    intakeLowerLeft.setInverted(Constants.Upper.UpperConstants.intakeLowerLeftInverted);
    
    intakeLowerRight.follow(intakeLowerLeft, Constants.Upper.UpperConstants.intakeLowerRightInverted);
    intakeUpper.setInverted(Constants.Upper.UpperConstants.intakeUpperInverted);
    
    tilterLeft.setInverted(Constants.Upper.UpperConstants.tilterLeftInverted);
    tilterRight.setControl(new Follower(Constants.Upper.UpperConstants.tilterLeft, Constants.Upper.UpperConstants.tilterRightInverted));

    tilterEncoder.setPosition(0);

    state = UpperState.start;
  }

  // intake
  public void setIntakeShooting(double speed) {
    intakeLowerLeft.set(speed);
  }

  public void setIntakeClaiming(double speed) {
    intakeUpper.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void stopIntakeRunning() {
    intakeLowerLeft.set(0);
    intakeLowerRight.set(0);
    intakeUpper.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public double getIntakeSpeed() {
    return (shooterLeftEncoder.getPosition() + shooterRightEncoder.getPosition()) / 2;
  }

  // tilter
  public void setTilter(double speed) {
    Constants.Upper.UpperVariables.tilterManualIsLocked = false;
    tilterLeft.set(speed);
  }

  public void lockTilter(double goal) {
    if(!Constants.Upper.UpperVariables.tilterManualIsLocked) { 
      Constants.Upper.UpperVariables.tilterLockedAngle = goal;
      Constants.Upper.UpperVariables.tilterManualIsLocked = true;
    }
  }

  public void stopTilter() {
    Constants.Upper.UpperVariables.tilterManualIsLocked = false;
    tilterLeft.set(0);
  }

  public double getTilterAngle() {
    return -tilterEncoder.getPosition().getValue() * 360;
  }

  public void resetTilterAngle() {
    tilterEncoder.setPosition(0);
  }

  // color sensor
  public double getHue() {
    frc.lib.math.Color color = new frc.lib.math.Color(
        colorSensor.getRed(),
        colorSensor.getGreen(),
        colorSensor.getBlue()
    );
    return color.getHue();
  }

  public boolean hasNote() {
      return Math.abs(getHue() - 30) < 10 ? true : false;
  }

  // state machine
  public UpperState getState() {
    return state;
  }

  public void setState(UpperState state) {
    this.state = state;
  }

  public void updateStates() {
    switch(state){
      case start:
        if(!tilterLimitSwitch.get()) tilterOutput = -Constants.Upper.UpperConstants.tilterAutoMaxSpeed;
        else tilterOutput = 0;
        intakeOutput = Constants.Upper.UpperConstants.intakeIdleSpeed;
        break;
      case auto:
        if(Constants.Vision.VisionVariables.tilterAutoAngle != 404) 
          tilterOutput = MathUtility.clamp(tilterPID.calculate(Constants.Vision.VisionVariables.tilterAutoAngle - getTilterAngle()), -Constants.Upper.UpperConstants.tilterAutoMaxSpeed, Constants.Upper.UpperConstants.tilterAutoMaxSpeed);
        else tilterOutput = MathUtility.clamp(tilterPID.calculate(Constants.Upper.UpperConstants.tilterIdleAngle - getTilterAngle()), -Constants.Upper.UpperConstants.tilterAutoMaxSpeed, Constants.Upper.UpperConstants.tilterAutoMaxSpeed);
        intakeOutput = Constants.Upper.UpperConstants.intakeShootingSpeed;
        Constants.Upper.UpperVariables.isAuto = true;
        Constants.Upper.UpperVariables.isAmp = false;
        break;
      case base:
        tilterOutput = MathUtility.clamp(tilterPID.calculate(Constants.Upper.UpperConstants.tilterBaseAngle - getTilterAngle()), -Constants.Upper.UpperConstants.tilterAutoMaxSpeed, Constants.Upper.UpperConstants.tilterAutoMaxSpeed);
        intakeOutput = Constants.Upper.UpperConstants.intakeShootingSpeed;
        Constants.Upper.UpperVariables.isAuto = false;
        Constants.Upper.UpperVariables.isAmp = false;
        break;
      case podium:
        tilterOutput = MathUtility.clamp(tilterPID.calculate(Constants.Upper.UpperConstants.tilterPodiumAngle - getTilterAngle()), -Constants.Upper.UpperConstants.tilterAutoMaxSpeed, Constants.Upper.UpperConstants.tilterAutoMaxSpeed);
        intakeOutput = Constants.Upper.UpperConstants.intakeShootingSpeed;
        Constants.Upper.UpperVariables.isAuto = false;
        Constants.Upper.UpperVariables.isAmp = false;
        break;
      case amp:
        tilterOutput = MathUtility.clamp(tilterPID.calculate(Constants.Upper.UpperConstants.tilterAmpAngle - getTilterAngle()), -Constants.Upper.UpperConstants.tilterAutoMaxSpeed, Constants.Upper.UpperConstants.tilterAutoMaxSpeed);
        intakeOutput = Constants.Upper.UpperConstants.intakeAmpingSpeed;
        Constants.Upper.UpperVariables.isAuto = false;
        Constants.Upper.UpperVariables.isAmp = true;
        break;
      case floor:
        tilterOutput = MathUtility.clamp(tilterPID.calculate(Constants.Upper.UpperConstants.tilterFloorAngle - getTilterAngle()), -Constants.Upper.UpperConstants.tilterAutoMaxSpeed, Constants.Upper.UpperConstants.tilterAutoMaxSpeed);
        intakeOutput = Constants.Upper.UpperConstants.intakeIdleSpeed;
        Constants.Upper.UpperVariables.isAuto = false;
        Constants.Upper.UpperVariables.isAmp = false;
        break;
      case manual:
        tilterOutput = Constants.Upper.UpperVariables.manualTilterSpeed;
        intakeOutput = Constants.Upper.UpperVariables.manualIntakeSpeed;
        if(tilterOutput == 0) lockTilter(getTilterAngle());
        Constants.Upper.UpperVariables.isAuto = false;
        Constants.Upper.UpperVariables.isAmp = false;
        break;
      case idle:
        tilterOutput = MathUtility.clamp(tilterPID.calculate(Constants.Upper.UpperConstants.tilterIdleAngle - getTilterAngle()), -Constants.Upper.UpperConstants.tilterAutoMaxSpeed, Constants.Upper.UpperConstants.tilterAutoMaxSpeed);
        intakeOutput = Constants.Upper.UpperConstants.intakeIdleSpeed;
        Constants.Upper.UpperVariables.isAuto = false;
        Constants.Upper.UpperVariables.isAmp = false;
        break;
    }
    setTilter(tilterOutput);
    setIntakeShooting(intakeOutput);
  }

  @Override
  public void periodic() {
    /* refreshes */
    updateStates();
    if(tilterLimitSwitch.get()) resetTilterAngle();
    Constants.Upper.UpperVariables.hasNote = hasNote();

    /* if-elses */
    if(Constants.Upper.UpperVariables.tilterManualIsLocked) {
      double currentAngle = getTilterAngle();
      double output = lockPID.calculate(Constants.Upper.UpperVariables.tilterLockedAngle - currentAngle);
      setTilter(output);
    }
    if((tilterOutput <= Constants.Upper.UpperConstants.tilterStabilizedSpeed && tilterOutput >= -Constants.Upper.UpperConstants.tilterStabilizedSpeed) && 
      (state == UpperState.auto || state == UpperState.base || state == UpperState.podium || state == UpperState.amp || state == UpperState.floor))
         Constants.Upper.UpperVariables.isInPlace = true;
    else Constants.Upper.UpperVariables.isInPlace = false;
    if(state == UpperState.floor) {
      if(hasNote()) setIntakeClaiming(Constants.Upper.UpperConstants.intakeClaimingSpeed);
      else setIntakeClaiming(0);
    }

    /* dashboard */
    SmartDashboard.putNumber("tilterSpeed", tilterOutput);
    SmartDashboard.putNumber("tilterDEG", getTilterAngle());
    SmartDashboard.putBoolean("tilterIsInPlace", Constants.Upper.UpperVariables.isInPlace);
    SmartDashboard.putNumber("intakeSpeed", intakeOutput);
    SmartDashboard.putString("state", getState().toString());
  }
}
