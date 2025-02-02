package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.EnumSet;
import java.util.Map;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
import frc.robot.Constants.ModuleConstants;

public class ArmSubsystem extends SubsystemBase {

    public enum ArmState {
        Start,
        CoralL4,
        CoralL3,
        CoralL2,
        CoralL1,
        AlgaeHuman,
        AlgaeL3,
        AlgaeL2,
        AlgaeL1,
        AlgaeShoot
    }

    private boolean isSim = false;
    private ArmState state = ArmState.AlgaeHuman;
    private double targetPosition = 0.0;
    private SparkFlex motor = null;
    private SparkFlexSim motorSim = null;
    private SparkClosedLoopController pid = null;
    private SparkFlexConfig config = new SparkFlexConfig();

    private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private double p = Constants.ArmConstants.P;
    private double i = Constants.ArmConstants.I;
    private double d = Constants.ArmConstants.D;

    public ArmSubsystem() {

        if (Constants.enableArm) {

            if (RobotBase.isReal()) {
                isSim = false;
            } else {
                isSim = true;
            }

            motor = new SparkFlex(Constants.ArmConstants.motor_id, MotorType.kBrushless);

		    if(isSim) {
			    motorSim = new SparkFlexSim(motor, DCMotor.getNeoVortex(1));
		    }

            setConfig();

		    pid = motor.getClosedLoopController();

            GenericEntry entry = Shuffleboard.getTab("Arm")
                .add("PID Controller P", p)
                .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
                .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
                .getEntry();

            networkTableInstance.addListener(
                networkTableInstance.getEntry(entry.getTopic().getName()),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    p = event.valueData.value.getDouble();
                    
                    setConfig();
                }
            );

            entry = Shuffleboard.getTab("Arm")
                .add("PID Controller I", i)
                .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
                .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
                .getEntry();

            networkTableInstance.addListener(
                networkTableInstance.getEntry(entry.getTopic().getName()),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    i = event.valueData.value.getDouble();
                    
                    setConfig();
                }
            );

            entry = Shuffleboard.getTab("Arm")
                .add("PID Controller D", d)
                .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
                .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
                .getEntry();

            networkTableInstance.addListener(
                networkTableInstance.getEntry(entry.getTopic().getName()),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    d = event.valueData.value.getDouble();
                    
                    setConfig();
                }
            );
        }
    }

    public void setDesiredState(ArmState state) {
        this.state = state;

        switch(state) {
            case AlgaeHuman:
                targetPosition = 0.0;
                break;
            case AlgaeL1:
                targetPosition = 0.0;
                break;
            case AlgaeL2:
                targetPosition = 0.0;
                break;
            case AlgaeL3:
                targetPosition = 0.0;
                break;
            case AlgaeShoot:
                targetPosition = 0.0;
                break;
            case CoralL1:
                targetPosition = Constants.ArmConstants.CoralL1;
                break;
            case CoralL2:
                targetPosition = Constants.ArmConstants.CoralL2;
                break;
            case CoralL3:
                targetPosition = Constants.ArmConstants.CoralL3;
                break;
            case CoralL4:
                targetPosition = Constants.ArmConstants.CoralL4;
                break;
            default:
                targetPosition = 0.0;
                break;
            
        }

        //drivePID.setReference(targetPosition, ControlType.kPosition);
    }

    @Override
	public void periodic() {

        if (Constants.enableArm) {
            pid.setReference(targetPosition, ControlType.kPosition);

            if (isSim) {
                motorSim.iterate(
                        // 0.1,
                        // desiredState.speedMetersPerSecond,
                        motor.getOutputCurrent(),
                        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                        0.02);
            }
        }
    }
    
    private void setConfig() {
        config = new SparkFlexConfig();

        config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(25)
            .velocityConversionFactor(25);
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
    }
}
