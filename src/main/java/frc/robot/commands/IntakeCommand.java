package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends SequentialCommandGroup {

    public IntakeCommand() {
        Intake intake = RobotContainer.m_intake;
        Indexing indexing = RobotContainer.m_indexing;
        Shooter shooter = RobotContainer.m_shooter;

        addRequirements(intake, indexing, shooter);

        final double intakeCurrentThreshold   = 10.0; // amps (i think)
        final double indexingCurrentThreshold = 10.0; // amps (i think)
        final double intakeDistance           = 50.0;  // meters 
        final double indexingDistance         = 0.3;  // meters 
        final double velocityThreshold        = 0.05; // meters / second

        addCommands(
            new InstantCommand(intake::stop),
            new InstantCommand(indexing::stop),
            new InstantCommand(shooter::stop),

            new InstantCommand(intake::intake),
            new WaitCommand(.5),
            new WaitUntilCommand(() -> intake.getCurrent() > intakeCurrentThreshold), // intake until it hits intake and intake gets a current spike
            new InstantCommand(() -> intake.intakeDistance(intakeDistance)), // take it in for a certain distance
            new WaitUntilCommand(() -> indexing.getCurrent() > indexingCurrentThreshold), // wait until it hits indexing and indexing gets a current spike
            new InstantCommand(() -> indexing.intakeDistance(indexingDistance)), // take it up for a certain distance
            new WaitCommand(0.5), // just in case it takes time to speed up
            new WaitUntilCommand(() -> indexing.getVelocity() < velocityThreshold), // wait until it has reached the setpoint
            new InstantCommand(intake::stop), // stop everything
            new InstantCommand(indexing::stop)
        );
        indexing.setLoaded(true);
    }
}
