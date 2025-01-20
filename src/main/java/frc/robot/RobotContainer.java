// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.controlConstants;
import frc.robot.subsystems.DriveTrain;
import swervelib.SwerveInputStream;

public class RobotContainer {

  DriveTrain driveTrain =  new DriveTrain(new File(Filesystem.getDeployDirectory(), "swerve"));
    XboxController driverXbox = new XboxController(controlConstants.driveController);

    //SWERVE
      SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveTrain.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(controlConstants.deadband)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
  


  public RobotContainer() {
    configureBindings();
  }

   /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings() {
    Command driveFieldOrientedDirectAngle      = driveTrain.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = driveTrain.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = driveTrain.driveFieldOriented(driveRobotOriented);
    driveTrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void setMotorBrake(boolean brake)
  {
    driveTrain.setMotorBrake(brake);
  }
}
