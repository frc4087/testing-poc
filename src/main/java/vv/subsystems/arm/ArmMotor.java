package vv.subsystems.arm;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import vv.config.ArmConfig;
import vv.config.SimulationConfig;
import vv.sims.TalonFXSim;

public class ArmMotor {
    private final TalonFX talon;
	private final Optional<TalonFXSim> sim;
	private final MotionMagicVoltage motorControl = new MotionMagicVoltage(0);

    public ArmMotor(ArmConfig.MotorConfig config, SimulationConfig simConfig) {
        var talonFXConfigs = new TalonFXConfiguration();
		talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        talonFXConfigs.Slot0.kS = 0.0;
		talonFXConfigs.Slot0.kG = 0.25;
		talonFXConfigs.Slot0.kV = 0.0;
		talonFXConfigs.Slot0.kP = 1.875;
		talonFXConfigs.Slot0.kI = 0.0;
		talonFXConfigs.Slot0.kD = 0.0;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = 160.0;
		talonFXConfigs.MotionMagic.MotionMagicAcceleration = 240.0;
		talonFXConfigs.MotionMagic.MotionMagicJerk = 3200.0;

        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = config.statorCurrentLimit().baseUnitMagnitude();
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = config.supplyCurrentLimit().baseUnitMagnitude();
		limitConfigs.SupplyCurrentLimitEnable = true;

		talon = new TalonFX(config.port());
		talon.getConfigurator().apply(talonFXConfigs, 0.050);
        talon.getConfigurator().apply(limitConfigs);
        talon.setNeutralMode(NeutralModeValue.Brake);
		talon.setPosition(0.0);

		var sysId = LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getKrakenX60Foc(1), 
            0.061,
            1
        );
        sim = Optional.ofNullable(Utils.isSimulation() ? new TalonFXSim(simConfig, talon, sysId, 1) : null);
	}

	/**
	 * @param relativeRotations from starting position
	 */
    public void setControlPosition(double relativeRotations) {
		talon.setControl(motorControl.withSlot(0).withPosition(relativeRotations));
    }

	/**
	 * @param speed in [-1, 1]
	 */
	public void setSpeed(double speed) {
		talon.set(speed);
	}

	public void reset() {
		talon.set(0.0);
		talon.setPosition(0.0);
	}


    public Optional<TalonFXSim> sim() {
        return sim;
    }

	public double getSetSpeed() {
        return talon.get();
    }

	public Angle getPosition() {
		talon.getPosition().refresh();
        return talon.getPosition().getValue();
    }
}
