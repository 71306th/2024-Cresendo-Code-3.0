// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.Upper.UpperConstants.UpperState;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Upper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TeleopControl2P extends Command {

  private final Swerve s_Swerve;
  private final Upper s_Upper;
  private final Controller s_Controller;

  private final XboxController driver;
  private final XboxController operator;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private boolean onePressField = false;
  private boolean onePressSlow = false;
  private boolean onePressRainbow = false;
  private boolean onePressManual = false;
  private boolean oneTimeGetTarget = false;
  private boolean oneTimeLostTarget = false;
  private boolean oneTimeLoad = false;
  private boolean oneTimeInPlace = false;
  private boolean lastHasTarget = false;
  private boolean lastLoad = false;
  private boolean lastInPlace = false;

  private double translationVal;
  private double strafeVal;
  private double rotationVal;
  private double leftDRumbleTime;
  private double rightDRumbleTime;
  private double leftORumbleTime;
  private double rightORumbleTime;

  public TeleopControl2P(Swerve s1, Upper s2, Controller control) {
    s_Swerve = s1;
    addRequirements(s_Swerve);
    s_Upper = s2;
    addRequirements(s_Upper);
    s_Controller = control;
    addRequirements(s_Controller);
    driver = s_Controller.getDriverController();
    operator = s_Controller.getOperatorController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driver.setRumble(RumbleType.kBothRumble, 0);
    operator.setRumble(RumbleType.kBothRumble, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Controlling Values */
    // Driver
    translationVal = Constants.Swerve.SwerveVariables.slow ?
        translationLimiter.calculate(
            MathUtil.applyDeadband(driver.getLeftY() * Constants.Swerve.SwerveConstants.slowRegulator, Constants.Joystick.axisDeadBand)) : 
        translationLimiter.calculate(
            MathUtil.applyDeadband(driver.getLeftY(), Constants.Joystick.axisDeadBand));  
    strafeVal = Constants.Swerve.SwerveVariables.slow ?
        strafeLimiter.calculate(
            MathUtil.applyDeadband(driver.getLeftX() * Constants.Swerve.SwerveConstants.slowRegulator, Constants.Joystick.axisDeadBand)) : 
        strafeLimiter.calculate(
            MathUtil.applyDeadband(driver.getLeftX(), Constants.Joystick.axisDeadBand)); 
    if(Constants.Vision.VisionVariables.hasTarget) {
        if(s_Upper.getState() == UpperState.auto) rotationVal = Constants.Swerve.SwerveVariables.chassisToSpeakerAngle;
        else if(s_Upper.getState() == UpperState.amp) rotationVal = Constants.Swerve.SwerveVariables.chassisToAmpAngle;
    } else {
        rotationVal = Constants.Swerve.SwerveVariables.slow ?
            rotationLimiter.calculate(
                MathUtil.applyDeadband(driver.getRightX() * Math.pow(Constants.Swerve.SwerveConstants.slowRegulator, 2), Constants.Joystick.axisDeadBand)) : 
            rotationLimiter.calculate(
                MathUtil.applyDeadband(driver.getRightX() * Constants.Swerve.SwerveConstants.slowRegulator, Constants.Joystick.axisDeadBand));  
    }

    // Operator
    Constants.Upper.UpperVariables.manualTilterSpeed = -MathUtil.applyDeadband(operator.getLeftY(), Constants.Joystick.axisDeadBand) * Constants.Upper.UpperConstants.tilterAutoMaxSpeed;
    Constants.Upper.UpperVariables.manualIntakeSpeed = -MathUtil.applyDeadband(operator.getRightY(), Constants.Joystick.axisDeadBand) * Constants.Upper.UpperConstants.intakeShootingSpeed;

    /* Buttons */
    // Driver
    if (driver.getLeftBumperPressed() && onePressField == false && Constants.Upper.UpperVariables.isAuto == false) {
      Constants.Swerve.SwerveVariables.fieldOriented = !Constants.Swerve.SwerveVariables.fieldOriented;
      onePressField = true;
    }else if (driver.getLeftBumperReleased()) {
      onePressField = false;
    }

    if (driver.getRightBumperPressed() && onePressSlow == false) {
      Constants.Swerve.SwerveVariables.slow = !Constants.Swerve.SwerveVariables.slow;
      onePressSlow = true;
    }else if (driver.getRightBumperReleased()) {
      onePressSlow = false;
    }

    if (driver.getStartButtonPressed() && onePressRainbow == false) {
      Constants.LED.LEDVariables.rainbow = !Constants.LED.LEDVariables.rainbow;
      onePressSlow = true;
    }else if (driver.getStartButtonReleased()) {
      onePressSlow = false;
    }

    if(driver.getBackButton()) s_Swerve.zeroGyro();

    // Operator
    if(operator.getAButton()) s_Upper.setState(UpperState.auto);
    if(operator.getBButton()) s_Upper.setState(UpperState.base);
    if(operator.getXButton()) s_Upper.setState(UpperState.podium);
    if(operator.getYButton()) s_Upper.setState(UpperState.floor);
    if(operator.getLeftBumper()) s_Upper.setState(UpperState.amp);
    if(operator.getRightBumper()) s_Upper.setState(UpperState.idle);

    if(operator.getStartButtonPressed() && onePressManual == false){
      Constants.Upper.UpperVariables.isManual = !Constants.Upper.UpperVariables.isManual;
      onePressManual = true;
    } else if(operator.getStartButtonReleased()) {
      onePressManual = false;
    }

    /* Drive */
    // Driver
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.SwerveConstants.maxSpeed),
        rotationVal * Constants.Swerve.SwerveConstants.maxAngularVelocity, Constants.Swerve.SwerveVariables.fieldOriented,
        true);

    // Operator
    if(Constants.Upper.UpperVariables.isManual) s_Upper.setState(UpperState.manual);

    /* Notifications */
    // Driver
    if(Constants.Vision.VisionVariables.hasTarget == true && lastHasTarget == false){
      if(!oneTimeGetTarget) {
        rightDRumbleTime = Timer.getFPGATimestamp() + Constants.Joystick.rumbleTime; 
        oneTimeGetTarget = true;
      }
      if(rightDRumbleTime >= Timer.getFPGATimestamp()) driver.setRumble(RumbleType.kRightRumble, 1);
      else {
        driver.setRumble(RumbleType.kRightRumble, 0); 
        oneTimeGetTarget = false;
        lastHasTarget = true;
      }
    } else if(Constants.Vision.VisionVariables.hasTarget == false && lastHasTarget == true) {
      if(!oneTimeLostTarget) {
        leftDRumbleTime = Timer.getFPGATimestamp() + Constants.Joystick.rumbleTime; 
        oneTimeLostTarget = true;
      }
      if(leftDRumbleTime >= Timer.getFPGATimestamp()) driver.setRumble(RumbleType.kLeftRumble, 1);
      else {
        driver.setRumble(RumbleType.kLeftRumble, 0); 
        oneTimeLostTarget = false;
        lastHasTarget = false;
      }
    }  

    // Operator
    if(s_Upper.hasNote() && !lastLoad) {
      if(!oneTimeLoad) {
        rightORumbleTime = Timer.getFPGATimestamp() + Constants.Joystick.rumbleTime; 
        oneTimeLoad = true;
      }
      if(rightORumbleTime >= Timer.getFPGATimestamp()) driver.setRumble(RumbleType.kRightRumble, 1);
      else {
        driver.setRumble(RumbleType.kRightRumble, 0); 
        oneTimeLoad = false;
        lastLoad = true;
      }
    } else lastLoad = false;

    if(Constants.Upper.UpperVariables.isInPlace && !lastInPlace) {
      if(!oneTimeInPlace) {
        leftORumbleTime = Timer.getFPGATimestamp() + Constants.Joystick.rumbleTime;
        oneTimeLoad = true;
      }
      if(leftORumbleTime >= Timer.getFPGATimestamp()) driver.setRumble(RumbleType.kLeftRumble, 1);
      else {
        operator.setRumble(RumbleType.kLeftRumble, 0);
        oneTimeInPlace = false;
        lastInPlace = true;
      }
    } else lastInPlace = false;
  }
}
