package vv.config;

import java.util.Objects;
import java.util.Properties;

import static vv.config.PropertyReaders.readDoubleProperty;
import static vv.config.PropertyReaders.readIntegerProperty;

public record ControllersConfig(
    Double rumbleIntensity,
    DriverConfig driver
) {
    public ControllersConfig {
        Objects.requireNonNull(rumbleIntensity, "Rumble intensity cannot be null");
        assert rumbleIntensity >= 0 && rumbleIntensity <= 1 : "Rumble intensity must be between 0 and 1";
        Objects.requireNonNull(driver, "Driver config cannot be null");
    }

    public static ControllersConfig fromProperties(Properties props) {
        return new ControllersConfig(
            readDoubleProperty(props, "controllers.rumble.intensity"),
            DriverConfig.fromProperties(props)
        );
    }

    public static record DriverConfig(
        Integer port,
        Double translationalDeadband,
        Double rotationalDeadband
    ) {
        public DriverConfig {
            Objects.requireNonNull(port, "Driver port cannot be null");
            Objects.requireNonNull(translationalDeadband, "Driver translationalDeadband cannot be null");
            Objects.requireNonNull(rotationalDeadband, "Driver rotationalDeadband cannot be null");
        }
    
        public static DriverConfig fromProperties(Properties props) {
            return new DriverConfig(
                readIntegerProperty(props, "controllers.driver.port"),
                readDoubleProperty(props, "controllers.driver.translational.deadband"),
                readDoubleProperty(props, "controllers.driver.rotational.deadband")
            );
        }
    }
}
