# Robot Configuration
This repository uses a swappable, centralized hardware
configuration system, defined in 
[RobotHardware.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RobotHardware.java).
This interacts with the [build-versions.sh](/build-versions.sh) shell script, which automatically builds debug and release packages for each hardware configuration.
This allows you to check for and retrieve abstracted subassemblies, without having to worry about the actual robot harware.
This system ties in to the revamped diagnostics system, allowing us to have one diagnostics opmode which will run only the tests that are supported by the current hardware configuration.

To retrieve subassemblies from the current hardware configuration, use the `RobotHardware.CURRENT` constant, which will be updated by the aforementioned script to reflect th current hardware configuration:
```java
// Check if we have a drive base
// _DO NOT_ use a specific drive base 
// implementation (e.g. MecanumDriveBase,
// ProgrammingBoardDriveBase).
// The correct implementation will be figured out by the
// configuration.
boolean hasDriveBase = RobotHardware.CURRENT.has(DriveBase.class);

// Get a drive base
// hardwareMap is the hardware map from the opmode
DriveBase driveBase = RobotHardware.CURRENT.get(DriveBase.class, hardwareMap);
```

To add a subassembly to a configuration, add it in `RobotHardware.java`:
```java
public interface RobotHardware {
    //...
    RobotHardware CONFIGURATION_NAME = () -> new SubassemblyAccessor<?>[] { // replace CONFIGURATION_NAME with the name of the configuration
            //...
            SubassemblyClass::new // Replace SubassemblyClass with your subassembly class
            //...
    };
    //...
}
```

To add a configuration, add another empty field, as above, with the subassemblies that you want.
To build that configuration, add a line to `build-versions.sh`:
```shell
matrix/build CONFIGURATION_NAME # replace CONFIGURATION_NAME with the name of the configuration
```