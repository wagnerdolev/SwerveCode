// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.DriveByJoysticks;
import frc.robot.SubSystems.SwerveSubSystem.Swerve;
import frc.robot.Utils.Vector2d;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("target angle", new Vector2d(1,1).theta());
    SmartDashboard.putNumber("current angle",Swerve.getInstance().modules[0].getSteeringAngle());
    SmartDashboard.putNumber("target speed", new Vector2d().mag());
    SmartDashboard.putNumber("current speed", Swerve.getInstance().modules[0].getDrvingSpeed());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
        Swerve.getInstance().modules[0].vectorToModule(new Vector2d(1,1));
  }

  @Override
  public void teleopPeriodic() {
    //DriveByJoysticks teleop = new DriveByJoysticks(null, null, null);
    //teleop.schedule();
    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
