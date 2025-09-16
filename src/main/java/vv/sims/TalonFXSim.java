package vv.sims;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import vv.config.SimulationConfig;

public class TalonFXSim {
    
    private final TalonFX talon;
    private final DCMotorSim sim;
    private final double loopPeriod; 

    public TalonFXSim(SimulationConfig config, TalonFX talon) {
        this.talon = talon;
        this.sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                0.001,
                1.0 
            ),
            DCMotor.getKrakenX60Foc(1)
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
