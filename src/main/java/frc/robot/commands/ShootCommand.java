package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Wrist;

public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(
        Indexing indexing, Shooter shooter
        // , Wrist wrist
    ) {
        addRequirements(indexing, shooter);

        final double windUpTime = 2.0; // seconds
        final double waitTime   = 1.0; // seconds

        addCommands(
            // Reset encoders
            new InstantCommand(shooter::shoot),
            new WaitCommand(windUpTime),
            new InstantCommand(indexing::shoot),
            new WaitCommand(waitTime),
            new InstantCommand(indexing::stop),
            new InstantCommand(shooter::stop)
        );
        indexing.setLoaded(false);
    }
}
