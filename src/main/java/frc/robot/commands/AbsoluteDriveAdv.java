// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;
import java.util.List;
import frc.robot.Constants;

public class AbsoluteDriveAdv extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY;
  private final DoubleSupplier  headingAdjust;
  private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight;
  private       boolean         resetHeading = false;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. Heading Adjust changes the current heading after being
   * multipied by a constant. The look booleans are shortcuts to get the robot to face a certian direction. Based off of
   * ideas in https://www.chiefdelphi.com/t/experiments-with-a-swerve-steering-knob/446172
   *
   * @param swerve        The swerve drivebase subsystem.
   * @param vX            DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY            DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive Y is towards the left wall when looking through
   *                      the driver station glass.
   * @param headingAdjust DoubleSupplier that supplies the component of the robot's heading angle that should be
   *                      adjusted. Should range from -1 to 1 with deadband already accounted for.
   * @param lookAway      Face the robot towards the opposing alliance's wall in the same direction the driver is
   *                      facing
   * @param lookTowards   Face the robot towards the driver
   * @param lookLeft      Face the robot left
   * @param lookRight     Face the robot right
   */
  public AbsoluteDriveAdv(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust,
                          BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft,
                          BooleanSupplier lookRight) 
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingAdjust = headingAdjust;
    this.lookAway = lookAway;
    this.lookTowards = lookTowards;
    this.lookLeft = lookLeft;
    this.lookRight = lookRight;

    addRequirements(swerve);

  }

  @Override
  public void initialize() {
    resetHeading = true;
  }

  @Override
  public void execute() {
    double headingX = 0;
    double headingY = 0;

    // These are written to allow combinations for 45 anfles
    // Face away from Drivers
    if(lookAway.getAsBoolean()) 
    {
      headingY = -1;
    }
    // Face Right
    if(lookRight.getAsBoolean()) 
    {
      headingX = 1;
    }
    // Face Left
    if(lookLeft.getAsBoolean()) 
    {
      headingX = -1;
    }
    // Face Drivers
    if(lookTowards.getAsBoolean()) 
    {
      headingY = 1;
    }

    // Prevents movement after Auto
    if(resetHeading) 
    {
      if(headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0) 
      {
        // Get the current Heading
        Rotation2d currentHeading = swerve.getHeading();

        // Set the current heading to the desired heading
        headingX = currentHeading.getSin();
        headingY = currentHeading.getCos();
      }
      // Don't reset heading again
      resetHeading = false;
    }

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY);

    // Limit velocity to prevent tippiness
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(), 
                                           Constants.LOOP_TIME,  Constants.ROBOT_MASS, List.of(Constants.CHASSIS), 
                                           swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0)
    {
      resetHeading = true;
      swerve.drive(translation, Constants.OperatorConstants.TURN_CONSTANT * -headingAdjust.getAsDouble(), true);
    }
  }


  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
