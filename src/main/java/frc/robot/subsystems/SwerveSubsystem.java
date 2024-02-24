// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;

import java.io.File;
//import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.PubSub;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import java.util.function.DoubleSupplier;
import com.kauailabs.navx.frc.AHRS;
import swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;

  public double maximumSpeed = Units.feetToMeters(13);


  public SwerveSubsystem(File directory) {

    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    //double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, 1);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
    //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
    //  The gear ratio is 6.75 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    //double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75, 1);

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false);
    AHRS navX = (AHRS)swerveDrive.swerveDriveConfiguration.imu.getIMU();
    
  }

    /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) 
    {
      return run(() -> {
        double xInput = Math.pow(translationX.getAsDouble(), 3);
        double yInput = Math.pow(translationY.getAsDouble(), 3);
        // Make it drive:
        driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, 
                                                                        headingX.getAsDouble(), 
                                                                        headingY.getAsDouble(), 
                                                                        swerveDrive.getYaw().getRadians(), 
                                                                        swerveDrive.getMaximumVelocity()));
      });
    }
      
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
      return run(() -> {
        // Make the robot move
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                                            translationY.getAsDouble() * swerveDrive.getMaximumVelocity()),
                          angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
                          true,
                          false);
      });
    }

    public void driveFieldOriented(ChassisSpeeds velocity)
    {
      swerveDrive.driveFieldOriented(velocity);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
      swerveDrive.drive(translation,
                        rotation,
                        fieldRelative,
                        false); // Open loop is disabled since it shouldn't be used most of the time.
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
