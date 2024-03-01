// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.sql.Driver;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AbsoluteDriveAdv;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final SwerveSubsystem m_driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  final CommandXboxController driverController = new CommandXboxController(OperatorConstants.XBOX_PORT);
  final CommandJoystick DriverStick = new CommandJoystick(OperatorConstants.JOYSTICK_PORT);
  
  public final Command slowDriveFieldOrientedAngularVelocity = m_driveBase.driveCommand(
        () -> -joystickScale.ScaleJoystick(DriverStick.getY(), OperatorConstants.JOYSTICK_XY_DEADBAND, OperatorConstants.JOYSTICK_EXPONENT) * 0.5,
        () -> -joystickScale.ScaleJoystick(driverController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND, OperatorConstants.RIGHT_X_EXPONENT) * 0.5,
        () -> joystickScale.ScaleJoystick(DriverStick.getTwist(), OperatorConstants.JOYSTICK_TWIST_DEADBAND, OperatorConstants.JOYSTICK_EXPONENT) * 0.4
    );

  public RobotContainer() {
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(m_driveBase,
                                                                   () -> -joystickScale.ScaleJoystick(DriverStick.getY(),
                                                                                                OperatorConstants.JOYSTICK_XY_DEADBAND,
                                                                                                OperatorConstants.JOYSTICK_EXPONENT),
                                                                   () -> -joystickScale.ScaleJoystick(DriverStick.getX(),
                                                                                                OperatorConstants.JOYSTICK_XY_DEADBAND,
                                                                                                OperatorConstants.JOYSTICK_EXPONENT),
                                                                   () -> -joystickScale.ScaleJoystick(DriverStick.getTwist(),
                                                                                                OperatorConstants.JOYSTICK_TWIST_DEADBAND,
                                                                                                OperatorConstants.JOYSTICK_TWIST_EXP),
                                                                   driverController.getHID()::getYButtonPressed,
                                                                   driverController.getHID()::getAButtonPressed,
                                                                   driverController.getHID()::getXButtonPressed,
                                                                   driverController.getHID()::getBButtonPressed);
  
      // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = m_driveBase.driveCommand(
        () -> -joystickScale.ScaleJoystick(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND, OperatorConstants.LEFT_XY_EXPONENT),
        () -> -joystickScale.ScaleJoystick(driverController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND, OperatorConstants.RIGHT_X_EXPONENT),
        () -> -driverController.getRightX(),
        () -> -driverController.getRightY()
    );

    Command driveFieldOrientedAngularVelocity = m_driveBase.driveCommand(
        () -> -joystickScale.ScaleJoystick(DriverStick.getY(), OperatorConstants.JOYSTICK_XY_DEADBAND, OperatorConstants.JOYSTICK_EXPONENT),
        () -> -joystickScale.ScaleJoystick(driverController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND, OperatorConstants.RIGHT_X_EXPONENT),
        () -> joystickScale.ScaleJoystick(DriverStick.getTwist(), OperatorConstants.JOYSTICK_TWIST_DEADBAND, OperatorConstants.JOYSTICK_EXPONENT) * 0.75
    );

    Command slowDriveFieldOrientedAngularVelocity = m_driveBase.driveCommand(
        () -> -joystickScale.ScaleJoystick(DriverStick.getY(), OperatorConstants.JOYSTICK_XY_DEADBAND, OperatorConstants.JOYSTICK_EXPONENT) * 0.5,
        () -> -joystickScale.ScaleJoystick(driverController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND, OperatorConstants.RIGHT_X_EXPONENT) * 0.5,
        () -> joystickScale.ScaleJoystick(DriverStick.getTwist(), OperatorConstants.JOYSTICK_TWIST_DEADBAND, OperatorConstants.JOYSTICK_EXPONENT) * 0.4
    );

    slowDriveFieldOrientedAngularVelocity.addRequirements(m_driveBase);

    m_driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  
  }

  private void configureBindings() 
  {
    Trigger slowDriveTRG = new Trigger(DriverStick.button(12));
    Trigger ejectTrigger = new Trigger(DriverStick.button(7));
    Trigger intakeTrigger = new Trigger(DriverStick.button(2));
    Trigger shootTrigger = new Trigger(DriverStick.button(1));

    slowDriveTRG.whileTrue(slowDriveFieldOrientedAngularVelocity);

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void setMotorBrake(boolean brake)
  {
    m_driveBase.setMotorBrake(brake);
  }

}
