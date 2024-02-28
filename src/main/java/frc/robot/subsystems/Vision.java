// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  private DoubleSubscriber tx;
  private DoubleSubscriber ty;
  private DoubleSubscriber tid;
  private DoubleArraySubscriber coordinate;

  private double x;
  private double y;
  private double[] coordinateArr;
  private double xDis;
  private double yDis;
  
  NetworkTable table;
  
  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getDoubleTopic("tx").subscribe(0.0);
    ty = table.getDoubleTopic("ty").subscribe(0.0);
    tid = table.getDoubleTopic("tid").subscribe(0.0);
    coordinate = table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[6]);
  }

  /* calculations */
  public double calculateTilterAngle() {
    if(Constants.Upper.UpperVariables.isAuto && Constants.Vision.VisionVariables.hasTarget) {
      double a = coordinateArr[2];
      double alpha = Constants.Swerve.SwerveVariables.chassisToSpeakerAngle + x;
      double b = Constants.Vision.VisionConstants.cameraToTilter;
      double beta = Constants.Upper.UpperConstants.angleArmIntake;
      double c = Constants.Upper.UpperConstants.armLength;
      double d = Constants.Vision.VisionConstants.centerIDToSpeakerZ;
      double sigma = Math.atan((d+a*Math.tan(alpha))/a);
      double f = a/Math.cos(sigma);
      return Math.asin(c*Math.sin(beta) / Math.sqrt(Math.pow(b, 2)+Math.pow(f, 2)-2*b*f*Math.cos(180-sigma)));
    } else return 404;
  }

  @Override
  public void periodic() {
    /* getting values */
    x = tx.get();
    y = ty.get();
    Constants.Vision.VisionVariables.id = tid.get();
    coordinateArr = coordinate.get();

    /* refreshing values */
    Constants.Vision.VisionVariables.visionX = x;
    Constants.Vision.VisionVariables.visionY = y + Constants.Vision.VisionConstants.cameraRoll;
    xDis = coordinateArr[2] + Constants.Vision.VisionConstants.cameraToBot / Math.cos(Constants.Swerve.SwerveVariables.chassisToSpeakerAngle + x);
    yDis = coordinateArr[2] + Constants.Vision.VisionConstants.cameraToBot / Math.cos(Constants.Swerve.SwerveVariables.chassisToSpeakerAngle + x) / Math.cos(Constants.Vision.VisionConstants.cameraRoll + y) + Constants.Vision.VisionConstants.centerIDToSpeakerZ;
    Constants.Vision.VisionVariables.botToSpeakerDis = Math.sqrt(Math.pow(xDis, 2) + Math.pow(yDis, 2));
    Constants.Vision.VisionVariables.tilterAutoAngle = calculateTilterAngle(); 

    if(Constants.Upper.UpperVariables.isAuto && (Constants.Vision.VisionVariables.id == 4 || Constants.Vision.VisionVariables.id == 7)) Constants.Vision.VisionVariables.hasTarget = true;
    else if(Constants.Upper.UpperVariables.isAmp && (Constants.Vision.VisionVariables.id == 5 || Constants.Vision.VisionVariables.id == 6)) Constants.Vision.VisionVariables.hasTarget = true;
    else Constants.Vision.VisionVariables.hasTarget = false;

    /* dashboard */
    SmartDashboard.putNumber("LimelightHoriDeg", x);
    SmartDashboard.putNumber("LimelightVertiDeg", y);
    SmartDashboard.putNumber("LimelightID", Constants.Vision.VisionVariables.id);
    SmartDashboard.putNumber("LimelightToTargetZ", coordinateArr[2]);
    SmartDashboard.putNumber("BotToSpeakerDistance", Constants.Vision.VisionVariables.botToSpeakerDis);
    SmartDashboard.putNumber("AutoCalculateTilterAngle", calculateTilterAngle());
    SmartDashboard.putBoolean("hasTarget", Constants.Vision.VisionVariables.hasTarget);
  }
}