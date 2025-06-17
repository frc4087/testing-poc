package vv;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Main {
    public static void main(String[] args) {
        System.out.println("Welcome to the VV application!");
        CommandScheduler.getInstance().run();
    }
}
