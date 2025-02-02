package frc.robot.subsystems;


import java.util.EnumSet;
import java.util.Map;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffectorSubsystem extends SubsystemBase {
    public enum EndEffectorState {
        Stopped,
        IntakeAlgaeFloor,
        IntakeCoralHumanElement,
        EjectAlgaeFloor,
        EjectCoral,
    }

    private boolean isSim = false;
    private EndEffectorState state = EndEffectorState.Stopped;
    private double targetVelocity1 = 0.0;
    private double targetVelocity2 = 0.0;
    private SparkFlex motor = null;
    private SparkFlexSim motorSim = null;
    private SparkClosedLoopController pid = null;
    private SparkFlexConfig config = new SparkFlexConfig();
    private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private double p = Constants.EndEffectorConstants.P;
    private double i = Constants.EndEffectorConstants.I;
    private double d = Constants.EndEffectorConstants.D;

    private SparkMax motor2 = null;
    private SparkMaxSim motor2Sim = null;
    private SparkClosedLoopController pid2 = null;
    private SparkMaxConfig config2 = new SparkMaxConfig();

    private boolean hasItem = false;

    public EndEffectorSubsystem() {
        if(Constants.enableEndEffector) {
            if (RobotBase.isReal()) {
                isSim = false;
            } else {
                isSim = true;
            }

            motor = new SparkFlex(Constants.EndEffectorConstants.motor_id, MotorType.kBrushless);
            motor2 = new SparkMax(Constants.EndEffectorConstants.motor2_id, MotorType.kBrushless);

            if(isSim) {
			    motorSim = new SparkFlexSim(motor, DCMotor.getNeoVortex(1));
                motor2Sim = new SparkMaxSim(motor2, DCMotor.getNeo550(1));
		    }

            setConfig();

		    pid = motor.getClosedLoopController();
            pid2 = motor2.getClosedLoopController();

            GenericEntry entry = Shuffleboard.getTab("EndEffector")
                .add("PID Controller P", p)
                .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
                .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
                .getEntry();

            networkTableInstance.addListener(
                //networkTableInstance.getEntry("/Shuffleboard/EndEffector/PID Controller P"),
                networkTableInstance.getEntry(entry.getTopic().getName()),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    p = event.valueData.value.getDouble();
                    
                    setConfig();
                }
            );

            entry = Shuffleboard.getTab("EndEffector")
                .add("PID Controller I", i)
                .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
                .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
                .getEntry();

            networkTableInstance.addListener(
                //networkTableInstance.getEntry("/Shuffleboard/EndEffector/PID Controller P"),
                networkTableInstance.getEntry(entry.getTopic().getName()),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    i = event.valueData.value.getDouble();
                    
                    setConfig();
                }
            );

            entry = Shuffleboard.getTab("EndEffector")
                .add("PID Controller D", d)
                .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
                .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
                .getEntry();

            networkTableInstance.addListener(
                //networkTableInstance.getEntry("/Shuffleboard/EndEffector/PID Controller P"),
                networkTableInstance.getEntry(entry.getTopic().getName()),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    d = event.valueData.value.getDouble();
                    
                    setConfig();
                }
            );
        }
    }

    public void setDesiredState(EndEffectorState state) {

        // Are we sending the same state again?  If so act like a toggle and stop
        if (this.state == state) {
            targetVelocity1 = Constants.EndEffectorConstants.StoppedMotor1;
            targetVelocity2 = Constants.EndEffectorConstants.StoppedMotor2;
        } else {

            switch (state) {
                case Stopped:
                    targetVelocity1 = Constants.EndEffectorConstants.StoppedMotor1;
                    targetVelocity2 = Constants.EndEffectorConstants.StoppedMotor2;
                    break;
                case IntakeAlgaeFloor:
                    targetVelocity1 = Constants.EndEffectorConstants.IntakeAlgaeFloorMotor1;
                    targetVelocity2 = Constants.EndEffectorConstants.IntakeAlgaeFloorMmotor2;
                    hasItem = false;
                    break;
                case IntakeCoralHumanElement:
                    targetVelocity1 = Constants.EndEffectorConstants.IntakeCoralHumanElementMotor1;
                    targetVelocity2 = Constants.EndEffectorConstants.IntakeCoralHumanElementMotor2;
                    hasItem = false;
                    break;
                case EjectAlgaeFloor:
                    targetVelocity1 = Constants.EndEffectorConstants.EjectAlgaeFloorMotor1;
                    targetVelocity2 = Constants.EndEffectorConstants.EjectAlgaeFloorMotor2;
                    hasItem = false;
                    break;
                case EjectCoral:
                    targetVelocity1 = Constants.EndEffectorConstants.EjectCoralMotor1;
                    targetVelocity2 = Constants.EndEffectorConstants.EjectCoralMotor2;
                    hasItem = false;
                    break;
                default:
                    targetVelocity1 = 0.0;
                    targetVelocity2 = 0.0;
                    break;
            }
        }

        this.state = state;

        // drivePID.setReference(targetPosition, ControlType.kPosition);
    }

    @Override
	public void periodic() {

        if (Constants.enableEndEffector) {
            pid.setReference(targetVelocity1, ControlType.kVelocity);
            pid2.setReference(targetVelocity2, ControlType.kVelocity);

            // Do we have the item?
            if(
                motor.getOutputCurrent() >= Constants.EndEffectorConstants.OutputCurrentLimitMotor1
                || motor2.getOutputCurrent() >= Constants.EndEffectorConstants.OutputCurrentLimitMotor2
            ) {
                // Looks like we got the item, so stop the end effector motors
                state = EndEffectorState.Stopped;
                hasItem = true;
            }

            if (isSim) {
                motorSim.iterate(
                        // 0.1,
                        // desiredState.speedMetersPerSecond,
                        motor.getOutputCurrent(),
                        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                        0.02
                    );

                motor2Sim.iterate(
                        motor2.getOutputCurrent(),
                        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                        0.02
                    );
            }
        }
    }

    private void setConfig() {
        // Vortex
        config = new SparkFlexConfig();

        config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
		    .pid(
			    p, 
                i, 
                d
			);
        config.signals.primaryEncoderPositionPeriodMs(5);

        motor.configure(
		    config, 
			ResetMode.kResetSafeParameters, 
			PersistMode.kPersistParameters
		);

        // Neo 550
        config2 = new SparkMaxConfig();

        config2.inverted(true)
            .idleMode(IdleMode.kBrake);
        config2.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        config2.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
			    p, 
                i, 
                d
			);

        config2.signals.primaryEncoderPositionPeriodMs(5);
    }

    public boolean hasItem() {
        return this.hasItem;
    }
}
