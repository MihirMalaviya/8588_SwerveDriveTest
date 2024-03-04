package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

public class AutonCommand extends SequentialCommandGroup {

    public AutonCommand(
        SwerveSubsystem driveSubsystem//, Intake intake, Indexing indexing, Shooter shooter, Wrist wrist
    ) {
        addRequirements(driveSubsystem);
        
        final double fwdSpeed = 0.25; // meters per second
        final double distance = 1.25; // distance

        addCommands(
            // Reset encoders
            // new InstantCommand(driveSubsystem::resetEncoders),
            driveSubsystem.run(() -> driveSubsystem.drive(new ChassisSpeeds(0, 0.25, 0))).withTimeout(fwdSpeed/distance)
        );
    }

}
