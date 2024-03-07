package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ScoringArea;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist;

public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand() {
        Intake intake = RobotContainer.m_intake;
        Indexing indexing = RobotContainer.m_indexing;
        Shooter shooter = RobotContainer.m_shooter;
        // Wrist wrist = RobotContainer.m_wrist;

        addRequirements(intake, indexing, shooter);

        final double windUpTime = 2.0; // seconds
        final double waitTime   = 1.0; // seconds

        addCommands(
            new InstantCommand(intake::stop), // reset everything just in case
            new InstantCommand(indexing::stop),
            new InstantCommand(shooter::stop),

            new InstantCommand(shooter::shoot), // wind up shooter
            // new InstantCommand(() -> wrist.setWristGoalCommand(
            //     RobotContainer.scoringArea == ScoringArea.SPEAKER 
            //     ? RobotContainer.speakerLookupTable.getAngleFromDistance(RobotContainer.distanceFromTarget) 
            //     : RobotContainer.ampLookupTable.getAngleFromDistance(RobotContainer.distanceFromTarget))),
            new WaitCommand(windUpTime), // wait till it is fast enough
            new InstantCommand(indexing::shoot), // feed in the note
            new WaitCommand(waitTime), // wait
            new InstantCommand(indexing::stop), // stop
            new InstantCommand(shooter::stop)
        );
        indexing.setLoaded(false);
    }
}
