package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.shooter.ShooterManagerSetReference;
import frc.robot.commands.turret.TrackTargetCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterManager;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.types.DistanceMap;
import frc.robot.utilities.Limelight;

public class ShootThreeThenDrive extends SequentialCommandGroup {

  public ShootThreeThenDrive(DrivetrainSubsystem drivetrain,
  FlywheelSubsystem flywheel, HoodSubsystem hood, TurretSubsystem turret,
  FeederSubsystem feeder, Limelight limelight, ShooterManager shootermanager,
  DistanceMap distanceMap) {
    super(
      new ParallelCommandGroup(
        new TrackTargetCommand(turret, drivetrain, limelight),

        new ShooterManagerSetReference(shootermanager, 
        distanceMap.getHoodDegrees(limelight.getTargetDistance()),
        distanceMap.getFlywheelRPM(limelight.getTargetDistance())),

        new RunCommand(
          () -> {
              flywheel.setVelocity(shootermanager.getFlywheelReference());
              hood.setHoodDegrees(shootermanager.getHoodReference());
          }, 
        flywheel,hood).withTimeout(8),

        new SequentialCommandGroup(
          new WaitCommand(3),
          new RunCommand(feeder::startFeeder, feeder).withTimeout(5)
        )
      ).withTimeout(8),

      new Drive(drivetrain, 0.6, 0).withTimeout(3)
    );
  }
}