// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro1;
  private final Pigeon2 gyro2;
  private final Pigeon2 gyro3;
  private final Pigeon2 gyro4;

  private SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;

  private Field2d field;

  double autoAimRotateAngle = 0, visionYawTotal = 0, counter = 1, RotationVal = 0, lastRotationVal = 0;
  int canCalculate = 0;
  double[] visionYawValue = new double[10];

  public Swerve() {
    gyro1 = new Pigeon2(Constants.Swerve.SwerveConstants.pigeon1, Constants.Robot.canbus);
    gyro2 = new Pigeon2(Constants.Swerve.SwerveConstants.pigeon2, Constants.Robot.canbus);
    gyro3 = new Pigeon2(Constants.Swerve.SwerveConstants.pigeon3, Constants.Robot.canbus);
    gyro4 = new Pigeon2(Constants.Swerve.SwerveConstants.pigeon4, Constants.Robot.canbus);
    zeroGyro();

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.SwerveConstants.swerveKinematics, getYaw(), pos);

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.SwerveConstants.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.SwerveConstants.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.SwerveConstants.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.SwerveConstants.Mod3.constants)
        };
 
    field = new Field2d();
  }

  public static SwerveModulePosition[] pos = {
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0))
  };

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.SwerveConstants.swerveKinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                    : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }


  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = mSwerveMods[0].getPosition();
    positions[1] = mSwerveMods[1].getPosition();
    positions[2] = mSwerveMods[2].getPosition();
    positions[3] = mSwerveMods[3].getPosition();
    return positions;
  }

  public void zeroGyro() {
    gyro1.setYaw(0);
    gyro2.setYaw(0);
    gyro3.setYaw(0);
    gyro4.setYaw(0);
  }

  public Rotation2d getYaw(){
    StatusSignal<Double> gyro1Yaw = gyro1.getYaw();
    StatusSignal<Double> gyro2Yaw = gyro2.getYaw();
    StatusSignal<Double> gyro3Yaw = gyro3.getYaw();
    StatusSignal<Double> gyro4Yaw = gyro4.getYaw();
    double averageAngle = (gyro1Yaw.getValue() + gyro2Yaw.getValue() + gyro3Yaw.getValue() + gyro4Yaw.getValue()) / 4;
    return (Constants.Swerve.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - averageAngle) : Rotation2d.fromDegrees(averageAngle);
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());
    Constants.Swerve.SwerveVariables.chassisToSpeakerAngle = getYaw().getDegrees()%360;
    Constants.Swerve.SwerveVariables.chassisToAmpAngle = 90 - Constants.Swerve.SwerveVariables.chassisToSpeakerAngle;

    SmartDashboard.putNumber("gyro ", getYaw().getDegrees());
    SmartDashboard.putBoolean("isOriented ", Constants.Swerve.SwerveVariables.fieldOriented);
    SmartDashboard.putBoolean("isSlow ", Constants.Swerve.SwerveVariables.slow);
    SmartDashboard.putBoolean("isAuto", Constants.Upper.UpperVariables.isAuto);

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getAngle().getDegrees());

      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
