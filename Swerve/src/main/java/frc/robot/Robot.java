// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.DriveByJoysticks;
import frc.robot.SubSystems.SwerveSubSystem.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Funcs;
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

    SmartDashboard.putNumber("Target angle", Math.toDegrees(new Vector2d(RobotContainer.chassis.getLeftX(), RobotContainer.chassis.getLeftY() * -1).rotateBy(Math.toRadians(270)).theta()));

    SmartDashboard.putNumber("Top right angle", Funcs.modulo(Swerve.getInstance().getModules()[0].getSteeringAngle(), 360));
    SmartDashboard.putNumber("Top left angle", Funcs.modulo(Swerve.getInstance().getModules()[1].getSteeringAngle(), 360));
    SmartDashboard.putNumber("Bottom left angle", Funcs.modulo(Swerve.getInstance().getModules()[2].getSteeringAngle(), 360));
    SmartDashboard.putNumber("Bottom right angle", Funcs.modulo(Swerve.getInstance().getModules()[3].getSteeringAngle(), 360));

    SmartDashboard.putNumber("Target speed", new Vector2d(RobotContainer.chassis.getLeftX(), RobotContainer.chassis.getLeftY() * -1).mag() * Consts.SwerveValues.MAX_DRIVE_SPEED);

    SmartDashboard.putNumber("Top right speed", Swerve.getInstance().getModules()[0].getDrivingSpeed());
    SmartDashboard.putNumber("Top left speed", Swerve.getInstance().getModules()[1].getDrivingSpeed());
    SmartDashboard.putNumber("Bottom left speed", Swerve.getInstance().getModules()[2].getDrivingSpeed());
    SmartDashboard.putNumber("Bottom right speed", Swerve.getInstance().getModules()[3].getDrivingSpeed());

    SmartDashboard.putNumber("Top right coder angle",Swerve.getInstance().getModules()[0].getCoderPos());
    SmartDashboard.putNumber("Top left coder angle",Swerve.getInstance().getModules()[1].getCoderPos());
    SmartDashboard.putNumber("Bottom left coder angle",Swerve.getInstance().getModules()[2].getCoderPos());
    SmartDashboard.putNumber("Bottom right coder angle",Swerve.getInstance().getModules()[3].getCoderPos());

    
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
  }

  @Override
  public void teleopPeriodic() {
    //Supplier<Double> joystickY = () -> RobotContainer.chassis.getLeftY() * -1;
    //DriveByJoysticks teleop = new DriveByJoysticks(() -> RobotContainer.chassis.getLeftX(), joystickY, () -> RobotContainer.chassis.getRightX());
    //teleop.schedule();

    Swerve.getInstance().getModules()[0].vectorToModule(new Vector2d(RobotContainer.chassis.getLeftX(), RobotContainer.chassis.getLeftY() * -1).rotateBy(Math.toRadians(90)));
    
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
