// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Controller extends SubsystemBase {
  
  public XboxController driver = null;
  public XboxController operator = null;

  public Controller() {}

  @Override
  public void periodic() {}

  public XboxController getDriverController() {
    if (driver == null) {
      driver = new XboxController(Constants.Joystick.kDriverControllerPort);
    }

    return driver;
  }

  public XboxController getOperatorController() {
    if (operator == null) {
      operator = new XboxController(Constants.Joystick.kOperatorControllerPort);
    }

    return operator;
  }
}