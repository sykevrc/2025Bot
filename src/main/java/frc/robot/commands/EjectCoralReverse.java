package frc.robot.commands;

import java.util.OptionalLong;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class EjectCoralReverse extends Command {

    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;
    private DriveSubsystem driveSubsystem;
    private EndEffectorSubsystem endEffectorSubsystem;

    private OptionalLong ejectTime = OptionalLong.empty();
    private boolean finished = false;

    public EjectCoralReverse(OptionalLong ejectTime) {

        this.elevatorSubsystem = RobotContainer.elevatorSubsystem;
        this.armSubsystem = RobotContainer.armSubsystem;
        this.driveSubsystem = RobotContainer.driveSubsystem;
        this.endEffectorSubsystem = RobotContainer.endEffectorSubsystem;

        addRequirements(elevatorSubsystem);
        addRequirements(armSubsystem);
        addRequirements(driveSubsystem);
        addRequirements(endEffectorSubsystem);

        this.ejectTime = ejectTime;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(ejectTime.isPresent()) {

            TimerTask task = new TimerTask() {
                public void run() {
                    System.out.println("EjectCoralReverse - stopping the eject");
                    endEffectorSubsystem.setDesiredState(EndEffectorState.Stopped);
                    elevatorSubsystem.setDesiredState(ElevatorState.CoralHuman);
                }
            };

            TimerTask task2 = new TimerTask() {
                public void run() {
                    System.out.println("EjectCoralReverse - setting the arm state");
                    armSubsystem.setDesiredState(ArmState.CoralHuman);
                }
            };

            TimerTask task3 = new TimerTask() {
                public void run() {
                    System.out.println("EjectCoralReverse - moving the robot back");
                    driveSubsystem.drive(0, -.3, 0);
                    finished = true;
                }
            };

            Timer timer = new Timer("Timer");
            //Timer timer2 = new Timer("Timer");
    
            timer.schedule(task, ejectTime.getAsLong());
            timer.schedule(task2, ejectTime.getAsLong() + 1000);
            timer.schedule(task3, ejectTime.getAsLong() + 3000);
            System.out.println("EjectCoralReverse - starting the eject");
            endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralBack);
            finished = false;
        } else {
            finished = true;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevatorSubsystem.setDesiredState(ElevatorState.CoralL4);
        armSubsystem.setDesiredState(ArmState.CoralL4);

        System.out.println("EjectCoralReverse::execute() called");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
