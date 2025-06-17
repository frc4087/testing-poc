# FRC-4087 Testing Proof of Concept
The intent of this project is to experiment with different ways to programatically test our code
without needing access to the robot or equipment (like a controller). We want to avoid "magic"
(ie. code we don't understand) and utilize our vendor libraries.

This includes:
- Using [Phoenix 6's built-in simulation capabilities](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/simulation/simulation-intro.html) 
- Moving TunerX generated configuration into properties files
- Creating utilities and patterns for common testing needs

## Dependencies
Install these before running the project:
- JDK 17
- [FRC VSCode 2025](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) 
- **new**: [VSCode Extension Pack for Java](vscode:extension/vscjava.vscode-java-pack): VSCode extension for Java (testing, autocomplete, etc/)

## Libraries
Apart from the standard FRC/WPI libraries, the following libraries are important to testing:
- [Awaitility](https://github.com/awaitility/awaitility): handles polling logic for async events (ex. command execution after)
- [jUnit](https://junit.org/junit5/): makes assertions expected behavior and tolerances

These are installed by Gradle.

## Notes
### Running Tests
You can run tests using Gradle's `test` task:
```bash
./gradlew test
```

The test runner from the **VSCode Extension Pack for Java** will add a "play" icon to testing files. You can click the button next
to a particular test method to run just that test or the button next to the class name to run all of the tests in the file (called a suite).

### Skipping Tests
If you want to build without worrying about skilling tests, add `-x test` to your Gradle command.
```bash
./gradlew build -x test
```