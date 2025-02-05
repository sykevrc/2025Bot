package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeWaitCommand extends Command{
    private IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
    private ElevatorSubsystem elevatorSubsystem = RobotContainer.elevatorSubsystem;

    public IntakeWaitCommand() {

        addRequirements(intakeSubsystem);
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevatorSubsystem.setTargetPosition(Constants.ElevatorConstants.CoralHuman);
        System.out.println("IntakeWaitCommand::execute() called");
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.hasItem();
    }
}
