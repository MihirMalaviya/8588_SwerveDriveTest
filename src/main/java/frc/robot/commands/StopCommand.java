package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StopCommand extends ParallelCommandGroup {

    public StopCommand() {
        Intake intake = RobotContainer.m_intake;
        Indexing indexing = RobotContainer.m_indexing;
        Shooter shooter = RobotContainer.m_shooter;

        addRequirements(intake, indexing, shooter);

        addCommands(
            new InstantCommand(intake::stop),
            new InstantCommand(indexing::stop),
            new InstantCommand(shooter::stop)
        );
    }
}
