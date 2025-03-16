// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Programmed by Amun Reddy of team 4643
package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.controlConstants;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import swervelib.SwerveInputStream;

public class RobotContainer {

  DriveTrain driveTrain = new DriveTrain(new File(Filesystem.getDeployDirectory(), "swerve"));
  XboxController driverXbox = new XboxController(controlConstants.driveController);
  public XboxController operatorContoller = new XboxController(controlConstants.operatorContoller);
  private Elevator m_elevatorSubsystem = new Elevator();
  Coral m_CoralSubsystem = new Coral();
  Climber m_ClimberSubsystem = new Climber();

  // SWERVE
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveTrain.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(controlConstants.deadband)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  // END OF SWERVE

  // ELEVATOR

  public RobotContainer() {
    NamedCommands.registerCommand("scorel4", scoreL4);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
   Command scoreL4 = new SequentialCommandGroup(new InstantCommand(m_elevatorSubsystem::setL4),  new InstantCommand(m_elevatorSubsystem::movePOS), new WaitCommand(4), new InstantCommand(m_CoralSubsystem::outtake), new WaitCommand(2), new InstantCommand(m_CoralSubsystem::stopCoralMotor));

  private void configureBindings() {
    
    Command driveFieldOrientedDirectAngle      = driveTrain.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = driveTrain.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = driveTrain.driveFieldOriented(driveRobotOriented);
    driveTrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    //Elevator Controls
    new JoystickButton(operatorContoller, XboxController.Button.kRightBumper.value)
    .onTrue(new InstantCommand(m_elevatorSubsystem::movePOS));

    new JoystickButton(operatorContoller, XboxController.Button.kB.value).onTrue(new InstantCommand(m_elevatorSubsystem::resetEnc));
    // Elevator Level set
    new Trigger(() -> operatorContoller.getPOV() == 0).onTrue(new InstantCommand(m_elevatorSubsystem::setL2));
    new Trigger(() -> operatorContoller.getPOV() == 90).onTrue(new InstantCommand(m_elevatorSubsystem::setL3));
    new Trigger(() -> operatorContoller.getPOV() == 180).onTrue(new InstantCommand(m_elevatorSubsystem::setL4));
    new Trigger(() -> operatorContoller.getPOV() == 270).onTrue(new InstantCommand(m_elevatorSubsystem::setL1));
   // new Trigger(m_elevatorSubsystem.limitSwitchPressedSup).onTrue(new InstantCommand(m_elevatorSubsystem::resetEnc));
    //Coral Controls
    new Trigger(() -> m_CoralSubsystem.intakeBeamBreakStatus()).onTrue(new SequentialCommandGroup(
      new InstantCommand(m_CoralSubsystem::startIntake), 
      new WaitUntilCommand(m_CoralSubsystem.coralBeamBreakStatus).withTimeout(5),
      new InstantCommand(m_CoralSubsystem::reverseIntake),
      new WaitUntilCommand(m_CoralSubsystem.coralBeamBreakStatusINV).withTimeout(2),
      new InstantCommand(m_CoralSubsystem::stopCoralMotor)
    ));


    new Trigger(() -> operatorContoller.getRightTriggerAxis() > 0.3).whileTrue(new InstantCommand(m_CoralSubsystem::outtake))
    .onFalse(new InstantCommand(m_CoralSubsystem::stopCoralMotor));

    new JoystickButton(operatorContoller, XboxController.Button.kLeftBumper.value).whileTrue(new SequentialCommandGroup( 
      new InstantCommand(m_CoralSubsystem::startIntake), 
      new WaitUntilCommand(m_CoralSubsystem.coralBeamBreakStatus),
      new InstantCommand(m_CoralSubsystem::slowIntake),
      new WaitUntilCommand(m_CoralSubsystem.coralBeamBreakStatusINV),
      new InstantCommand(m_CoralSubsystem::stopCoralMotor))).onFalse(new InstantCommand(m_CoralSubsystem::stopCoralMotor));

      //Climber Controlls
  new JoystickButton(operatorContoller, XboxController.Button.kY.value).onTrue(new InstantCommand(m_ClimberSubsystem::climb));
  
  new JoystickButton(operatorContoller, XboxController.Button.kA.value).onTrue(new InstantCommand(m_ClimberSubsystem::deClimb));

  new JoystickButton(operatorContoller, XboxController.Button.kX.value).onTrue(new InstantCommand(m_elevatorSubsystem::setIntake));
  }
  //DELETE AT SOME POINT
  public void manualElevator(){
    m_elevatorSubsystem.manualControl(operatorContoller.getRawAxis(2)*-1,false);

  }

  public void setMotorBrake(boolean brake) {
    driveTrain.setMotorBrake(brake);
  }

  public void robotContainerPerodic() {
    SmartDashboard.putNumber("Elevator Encoder Position", m_elevatorSubsystem.getEnc());
    SmartDashboard.putNumber("Intake Beam Break Value", m_CoralSubsystem.coroalBeamBreakStatusINT());
    SmartDashboard.putBoolean("FrontBeamBreak",  m_CoralSubsystem.coralBeamBreakStatus());
  }
  //Path planner

  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
}
