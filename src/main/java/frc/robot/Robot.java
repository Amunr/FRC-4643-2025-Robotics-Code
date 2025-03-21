  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot;

  import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.wpilibj2.command.Command;
  import edu.wpi.first.wpilibj2.command.CommandScheduler;

  public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
      m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
      CommandScheduler.getInstance().run();
    m_robotContainer.robotContainerPerodic();
    }

    @Override
    public void disabledInit() {
      m_robotContainer.setMotorBrake(true);

    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
      m_robotContainer.setMotorBrake(false);
      m_robotContainer.driveTrain.setupPathPlanner();
      
      m_autonomousCommand = m_robotContainer.getAutonomousCommand("Davis");

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
       m_robotContainer.setMotorBrake(false);
       m_robotContainer.driveTrain.resetOdometry(new Pose2d(new Translation2d(0,0), new Rotation2d(0,0)));

    }

    @Override
    public void teleopPeriodic() {
      m_robotContainer.manualElevator();
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
