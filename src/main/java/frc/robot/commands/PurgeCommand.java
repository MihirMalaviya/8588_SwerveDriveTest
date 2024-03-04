package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PurgeCommand extends SequentialCommandGroup {

    public PurgeCommand(
        Intake intake, Indexing indexing, Shooter shooter
    ) {
        addRequirements(intake, indexing, shooter);

        final double purgeTime = 3.0; // seconds

        addCommands(
            new InstantCommand(intake::intake),
            new InstantCommand(indexing::intake),
            new InstantCommand(shooter::intake),
            new WaitCommand(purgeTime),
            new InstantCommand(intake::stop),
            new InstantCommand(indexing::stop),
            new InstantCommand(shooter::stop)
        );
        indexing.setLoaded(false);
    }
}
