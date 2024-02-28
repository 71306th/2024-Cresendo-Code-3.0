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
public class TeleopControl1P extends Command {

  private final Swerve s_Swerve;
  private final Upper s_Upper;
  private final Controller s_Controller;

  private final XboxController driver;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private boolean onePressField = false;
  private boolean onePressSlow = false;
  private boolean onePressRainbow = false;
  private boolean oneTimeTarget = false;
  private boolean oneTimeLoad = false;
  private boolean lastHasTarget = false;
  private boolean lastLoad = false;

  private double translationVal;
  private double strafeVal;
  private double rotationVal;
  private double leftRumbleTime;
  private double rightRumbleTime;

  public TeleopControl1P(Swerve s1, Upper s2, Controller control) {
    s_Swerve = s1;
    addRequirements(s_Swerve);
    s_Upper = s2;
    addRequirements(s_Upper);
    s_Controller = control;
    addRequirements(s_Controller);
    driver = s_Controller.getDriverController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driver.setRumble(RumbleType.kBothRumble, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Driving Values */
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

    /* Buttons */
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

    if(driver.getBackButton()) s_Swerve.zeroGyro();
    if(driver.getStartButtonPressed()) s_Upper.setIntakeClaiming(Constants.Upper.UpperConstants.intakeClaimingSpeed);
    if(driver.getStartButtonReleased()) s_Upper.setIntakeClaiming(0);
    if(driver.getAButton()) s_Upper.setState(UpperState.auto);
    if(driver.getBButton()) s_Upper.setState(UpperState.base);
    if(driver.getXButton()) s_Upper.setState(UpperState.podium);
    if(driver.getYButton()) s_Upper.setState(UpperState.floor);
    if(driver.getLeftTriggerAxis() >= 0.5) s_Upper.setState(UpperState.amp);
    if(driver.getRightTriggerAxis() >= 0.5) s_Upper.setState(UpperState.idle);

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.SwerveConstants.maxSpeed),
        rotationVal * Constants.Swerve.SwerveConstants.maxAngularVelocity, Constants.Swerve.SwerveVariables.fieldOriented,
        true);

    /* Notifications */
    if((Constants.Vision.VisionVariables.hasTarget == true && lastHasTarget == false) || (Constants.Vision.VisionVariables.hasTarget == false && lastHasTarget == true)){
      if(!oneTimeTarget) {
        rightRumbleTime = Timer.getFPGATimestamp() + Constants.Joystick.rumbleTime; 
        oneTimeTarget = true;
      }
      if(rightRumbleTime >= Timer.getFPGATimestamp()) driver.setRumble(RumbleType.kRightRumble, 1);
      else {
        driver.setRumble(RumbleType.kBothRumble, 0); 
        oneTimeTarget = false;
        lastHasTarget = !lastHasTarget;
      }
    }

    if(s_Upper.hasNote() && !lastLoad){
      if(!oneTimeLoad) {
        leftRumbleTime = Timer.getFPGATimestamp() + Constants.Joystick.rumbleTime; 
        oneTimeLoad = true;
      }
      if(leftRumbleTime >= Timer.getFPGATimestamp()) driver.setRumble(RumbleType.kLeftRumble, 1);
      else {
        driver.setRumble(RumbleType.kBothRumble, 0); 
        oneTimeLoad = false;
        lastLoad = true;
      }
    } else lastLoad = false;
  }
}
