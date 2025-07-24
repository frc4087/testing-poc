package vv.config;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Properties;

public class VVConfig {

    private final DrivetrainConfig drivetrainConfig; 
    private final PIDConfig pid;
    private final SimulationConfig simulation;
    private final ControllersConfig controllers;

    private VVConfig(String filePath) {
        var props = readFileConfig(filePath);
        this.drivetrainConfig = DrivetrainConfig.fromProperties(props);
        this.pid = PIDConfig.fromProperties(props);
        this.simulation = SimulationConfig.fromProperties(props);
        this.controllers = ControllersConfig.fromProperties(props);
    }

    public DrivetrainConfig drivetrain() {
        return this.drivetrainConfig;
    }

    public PIDConfig pid() {
        return this.pid;
    }

    public SimulationConfig simulation() {
        return this.simulation;
    }

    public ControllersConfig controllers() {
        return this.controllers;
    }

    public static VVConfig readFromPath(String filePath) {
        return new VVConfig(filePath);
    }

    private static Properties readFileConfig(String path) {
        var properties = new Properties();
        try {
            var reader = new FileReader(path);
            properties.load(reader);
            return properties;
        } catch (FileNotFoundException e) {
            throw new Error("Unable to find property file: %s".formatted(e.getMessage()));
        } catch (IOException e) {
            throw new Error("IOException when reading property file: %s".formatted(e.getMessage()));
        }
    }

}
