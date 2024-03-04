package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends SequentialCommandGroup {

    public IntakeCommand(
        Intake intake, Indexing indexing
    ) {
        addRequirements(intake, indexing);

        final double intakeCurrentThreshold   = 20.0; // amps (i think)
        final double indexingCurrentThreshold = 20.0; // amps (i think)
        final double intakeDistance           = 1.0;  // meters 
        final double indexingDistance         = 0.3;  // meters 
        final double velocityThreshold        = 0.05; // meters / second

        addCommands(
            new InstantCommand(intake::intake),
            new WaitUntilCommand(() -> intake.getCurrent() > intakeCurrentThreshold), // intake until it hits intake and intake gets a current spike
            new InstantCommand(() -> intake.intakeDistance(intakeDistance)), // take it in for a certain distance
            new WaitUntilCommand(() -> indexing.getCurrent() > indexingCurrentThreshold), // wait until it hits indexing and indexing gets a current spike
            new InstantCommand(() -> indexing.intakeDistance(indexingDistance)), // take it up for a certain distance
            new WaitCommand(0.5), // just in case it takes time to speed up
            new WaitUntilCommand(() -> indexing.getVelocity() < velocityThreshold), // wait until it has reached the setpoint
            new InstantCommand(intake::stop), // stop everything
            new InstantCommand(indexing::stop)
        );
    }
}
