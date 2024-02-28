// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopControl1P;
import frc.robot.commands.TeleopControl2P;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Upper;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Swerve m_Swerve = new Swerve();
  private final Upper m_Upper = new Upper();
  private final Vision m_Vision = new Vision();
  private final LED m_LED = new LED();
  private final Controller m_Controller = new Controller();

  private final TeleopControl1P tele1p = new TeleopControl1P(m_Swerve, m_Upper, m_Controller);
  private final TeleopControl2P tele2p = new TeleopControl2P(m_Swerve, m_Upper, m_Controller);

  public RobotContainer() {

    m_Swerve.setDefaultCommand(tele1p);
    m_Upper.setDefaultCommand(tele1p);
    m_Vision.setDefaultCommand(null);
    m_LED.setDefaultCommand(null);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
