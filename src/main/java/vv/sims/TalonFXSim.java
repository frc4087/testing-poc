package vv.sims;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import vv.config.SimulationConfig;

public class TalonFXSim {
    
    protected final TalonFX talon;
    protected final DCMotorSim sim;
    protected final double loopPeriod; 

    public TalonFXSim(SimulationConfig config, TalonFX talon, LinearSystem<N2, N1, N2> sysId, Integer numMotors) {
        this.talon = talon;

        this.sim = new DCMotorSim(
            sysId,
            DCMotor.getKrakenX60Foc(numMotors)
        );
        this.loopPeriod = config.simLoopPeriodFreq().asPeriod().baseUnitMagnitude();
        talon.getDutyCycle().setUpdateFrequency(config.signalUpdateFreq());
    }

    public void updateState() {
        var simState = talon.getSimState();
        
        // Set supply voltage
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        // Get commanded voltage and apply to sim
        var motorVoltage = simState.getMotorVoltageMeasure().in(Volts);
        sim.setInputVoltage(motorVoltage);
        sim.update(this.loopPeriod);
        
        // Apply simulated results back to TalonFX
        simState.setRawRotorPosition(Units.radiansToRotations(sim.getAngularPositionRad()));
        simState.setRotorVelocity(Units.radiansToRotations(sim.getAngularVelocityRadPerSec()));
    }

}
