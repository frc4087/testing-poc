package vv.subsystems.roller;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;

import vv.config.RollerConfig;
import vv.config.SimulationConfig;
import vv.sims.TalonFXSim;

public class RollerMotor {
    
    private final TalonFX talon;
    private final Optional<TalonFXSim> sim;
    private final Double intakeSpeed;
    private final Double outputSpeed;
    private final Double idleSpeed;

    public RollerMotor(RollerConfig.MotorConfig config, SimulationConfig simConfig) {
        talon = new TalonFX(config.port());
        intakeSpeed = config.intakeSpeed();
        outputSpeed = config.outputSpeed();
        idleSpeed = config.idleSpeed();

        sim = Optional.ofNullable(Utils.isSimulation() ? new TalonFXSim(simConfig, talon) : null);
    }

    public void intake() {
        talon.set(intakeSpeed);
    }

    public void output() {
        talon.set(outputSpeed);
    }

    public void idle() {
        talon.set(idleSpeed);
    }

    public double getSetSpeed() {
        return talon.get();
    }

    public Optional<TalonFXSim> sim() {
        return sim;
    }
}
