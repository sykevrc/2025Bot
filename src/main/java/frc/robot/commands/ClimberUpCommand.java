package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class ClimberUpCommand extends Command {

    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;
    private boolean finished = false;
    
    public ClimberUpCommand() {
        this.elevatorSubsystem = RobotContainer.elevatorSubsystem;
        this.armSubsystem = RobotContainer.armSubsystem;
        addRequirements(elevatorSubsystem);
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finished = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if(Constants.kEnableElevator) {
            this.elevatorSubsystem.setDesiredState(ElevatorState.ClimberUp);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}
