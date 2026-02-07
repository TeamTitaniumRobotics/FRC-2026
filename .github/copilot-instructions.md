# GitHub Copilot Instructions for FRC-2026

⚠️ **CRITICAL - READ THIS FIRST** ⚠️
═══════════════════════════════════════════════════════════════════════════════

🛑 **MANDATORY REQUIREMENT - NO EXCEPTIONS** 🛑

Before answering ANY question about:
- WPILib APIs, classes, methods, or syntax
- Robot code (commands, subsystems, autonomous)
- FRC programming patterns or best practices
- Command-based programming
- Any WPILib framework functionality

YOU MUST:
1. **STOP** - Do not answer from memory or training data
2. **PREFER** - Use the `frc-docs` MCP server tools defined in `.github/copilot-mcp.json` to query the WPILib documentation FIRST, if those tools are available in your environment
3. **FALLBACK** - If the `frc-docs` MCP tools are not available, consult the official WPILib documentation at https://docs.wpilib.org/ using your browsing capabilities or prior knowledge of its structure
4. **VERIFY** - Base your answer on the retrieved official WPILib documentation (from the MCP tools or docs.wpilib.org)
5. **CITE** - Explicitly reference the official WPILib documentation (tool output and/or docs.wpilib.org URLs) in your answer

This is NOT optional. It is a MANDATORY requirement that ALL WPILib-related answers be grounded in and consistent with the official WPILib documentation.

═══════════════════════════════════════════════════════════════════════════════

## Project Overview

This is Team Titanium Robotics (Team 1986) FRC (FIRST Robotics Competition) 2026 robot code. The repository contains Java code for controlling a competition robot using WPILib's command-based framework with AdvantageKit logging and simulation support.

**Repository Stats:**
- **Size:** ~1MB (compact codebase)
- **Language:** Java 17
- **Files:** 48 Java source files
- **Team Number:** 1986

## Project Configuration

- **WPILib Version:** 2026.2.1
- **Java Version:** 17 (OpenJDK 17.0.18)
- **Gradle Version:** 8.11
- **Build Tool:** Gradle with GradleRIO plugin
- **Main Class:** `org.teamtitanium.Main`
- **Programming Language:** Java
- **Target Platform:** roboRIO (FRC Robot Controller)

## Build & Validation Commands

### Prerequisites
**CRITICAL:** Always ensure `gradlew` has execute permissions before running any Gradle commands:
```bash
chmod +x gradlew
```

### Build Commands

**Build the project (compile + run tests):**
```bash
./gradlew build
```
- This command compiles code, runs Spotless formatter, and executes tests
- **Expected time:** 30-120 seconds (first run slower due to dependency downloads)
- **Network requirement:** Requires internet access to download dependencies from:
  - `frcmaven.wpi.edu` (WPILib dependencies)
  - `maven.ctr-electronics.com` (Phoenix 6 / CTRE libraries)
  - `file.tavsys.net` (Sleipnir optimization library)
  - `shenzhen-robotics-alliance.github.io` (Maple-Sim simulation)
  - Maven Central (various dependencies)

**Clean build artifacts:**
```bash
./gradlew clean
```

**Deploy to robot:**
```bash
./gradlew deploy
```
- Deploys code to the roboRIO
- Team number (1986) is loaded from `.wpilib/wpilib_preferences.json`

### Code Formatting

**CRITICAL - Code formatting is automatically applied on build:**
- Spotless formatter runs **automatically** before every compilation via `compileJava.dependsOn spotlessApply`
- Uses Google Java Format (AOSP style)
- Applies to `.java`, `.gradle`, `.xml`, `.md`, and `.gitignore` files

**Check formatting without auto-fixing:**
```bash
./gradlew spotlessCheck
```
- This is what CI uses for PRs (see `.github/workflows/formatter.yml`)
- **Will fail if code is not properly formatted**

**Manually apply formatting:**
```bash
./gradlew spotlessApply
```
- Usually not needed since it runs automatically on build

### Testing

**Run tests:**
```bash
./gradlew test
```
- Uses JUnit 5 (Jupiter)
- Test configuration in `.vscode/settings.json` specifies working directory as `build/jni/release`

**Check Gradle version:**
```bash
./gradlew --version
```

### Simulation

**Run robot simulator with GUI:**
```bash
./gradlew simulateJava
```
- Launches WPILib simulation GUI
- Desktop support is enabled (`includeDesktopSupport = true` in build.gradle)

**AdvantageKit replay watch (auto-reload logs):**
```bash
./gradlew replayWatch
```

### Version File Generation

The build automatically generates `BuildConstants.java` with git metadata:
- Generated in `src/main/java/org/teamtitanium/BuildConstants.java`
- **This file is gitignored** - do not commit it
- Contains: git SHA, branch, build date, dirty status

## Continuous Integration (CI/CD)

### GitHub Actions Workflows

**1. Build Workflow (`.github/workflows/build.yml`)**
- **Triggers:** Push and pull requests to any branch
- **Container:** `wpilib/roborio-cross-ubuntu:2024-22.04`
- **Steps:**
  1. Checkout code
  2. Add repo to git safe directories
  3. Grant execute permission to gradlew
  4. Run `./gradlew build`

**2. Formatter Workflow (`.github/workflows/formatter.yml`)**
- **Triggers:** Pull requests to any branch
- **Container:** `wpilib/roborio-cross-ubuntu:2024-22.04`
- **Steps:**
  1. Checkout code
  2. Add repo to git safe directories
  3. Grant execute permission to gradlew
  4. Run `./gradlew spotlessCheck`
- **IMPORTANT:** This will fail if code is not properly formatted

### Common CI Failure Causes

1. **Formatting errors:** Code not formatted with Google Java Format
   - **Fix:** Run `./gradlew spotlessApply` before committing
2. **Build errors:** Compilation failures
   - **Fix:** Run `./gradlew build` locally first
3. **Network errors:** Dependencies cannot be downloaded
   - **Note:** This is expected in sandboxed environments; CI has network access

## Project Architecture

### Directory Structure

```
FRC-2026/
├── .github/
│   └── workflows/          # CI/CD workflows (build.yml, formatter.yml)
├── .vscode/                # VSCode settings and launch configurations
├── .wpilib/                # WPILib preferences (team number, year)
├── gradle/                 # Gradle wrapper files
├── src/main/
│   ├── deploy/            # Files deployed to robot (configs, paths, etc.)
│   └── java/org/teamtitanium/
│       ├── Main.java              # Entry point (starts Robot)
│       ├── Robot.java             # Main robot class (extends LoggedRobot)
│       ├── RobotState.java        # Robot state management
│       ├── MechanismVisualizer.java  # Visualization for mechanisms
│       ├── autos/                 # Autonomous routines
│       ├── commands/              # Command implementations
│       ├── optimization/          # Shooter optimization algorithms
│       ├── subsystems/
│       │   ├── Leds.java         # LED subsystem
│       │   ├── shooter/          # Shooter mechanisms (flywheel, hood, turret)
│       │   │   ├── flywheel/
│       │   │   ├── hood/
│       │   │   └── turret/
│       │   └── swerve/           # Swerve drive subsystem
│       │       ├── Swerve.java
│       │       ├── SwerveModule.java
│       │       ├── GyroIO*.java
│       │       ├── SwerveModuleIO*.java
│       │       └── PhoenixOdometryThread.java
│       └── utils/                # Utility classes
│           ├── Constants.java    # Global constants
│           ├── TunerConstants.java  # Tuner-generated constants
│           ├── FieldConstants.java  # Field geometry
│           ├── LoggedTunable*.java  # Tunable values
│           ├── PhoenixUtil.java  # Phoenix device utilities
│           └── swerve/           # Swerve-specific utilities
├── vendordeps/            # Vendor library dependencies (JSON files)
│   ├── AdvantageKit.json
│   ├── Phoenix6-26.1.0.json
│   ├── SleipnirJava.json
│   ├── WPILibNewCommands.json
│   └── maple-sim.json
├── build.gradle           # Main build configuration
├── settings.gradle        # Gradle settings
├── gradlew / gradlew.bat  # Gradle wrapper scripts
└── simgui-ds.json        # Simulator driver station config
```

### Code Organization Pattern

This project follows **AdvantageKit's IO layer pattern**:
- Each subsystem has an interface (e.g., `SwerveModuleIO`, `FlywheelIO`)
- Multiple implementations: `*IOTalonFX` (real hardware), `*IOSim` (simulation), empty interface (replay)
- Subsystem classes accept IO implementations via constructor injection
- Enables hardware abstraction and comprehensive logging

### Key Files Reference

**Root Directory:**
- `build.gradle` - Build configuration, dependencies, plugins, deploy settings
- `.gitignore` - Excludes build/, .gradle/, BuildConstants.java, logs/, etc.
- `.wpilib/wpilib_preferences.json` - Team number (1986), project year (2026)

**Configuration Files:**
- `.vscode/settings.json` - Java formatter settings, test config, import preferences
- `.vscode/launch.json` - Debug/run configurations for WPILib
- `vendordeps/*.json` - Third-party library dependencies

**Source Files:**
- `src/main/java/org/teamtitanium/Main.java` - Entry point, calls `RobotBase.startRobot(Robot::new)`
- `src/main/java/org/teamtitanium/Robot.java` - Main robot class with periodic methods
- `src/main/java/org/teamtitanium/utils/Constants.java` - Mode enum (REAL/SIM/REPLAY), global constants

## Dependencies & Vendor Libraries

**Core WPILib (2026.2.1):**
- wpilibj, wpimath, ntcore, cscore, hal, wpinet, wpiutil
- wpilibNewCommands (command-based framework)
- apriltag, wpiunits, epilogue

**Vendor Libraries:**
1. **AdvantageKit (26.0.0)** - Comprehensive logging and replay
   - Used extensively: Robot extends `LoggedRobot`
2. **Phoenix 6 (26.1.0)** - CTRE motor controllers and sensors
   - TalonFX motor controllers, Pigeon 2 IMU, CANivore
3. **Sleipnir (2026.0.0)** - Optimization library
4. **Maple-Sim (0.4.0-beta)** - Advanced physics simulation
5. **WPILibNewCommands** - Command-based programming

**Build Plugins:**
- `edu.wpi.first.GradleRIO` (2026.2.1) - FRC build system
- `com.diffplug.spotless` (6.20.0) - Code formatting
- `com.peterabeles.gversion` (1.10) - Version file generation
- `io.freefair.lombok` (6.6.1) - Lombok support

## Important Implementation Details

### Mode Selection
Robot mode is determined in `Constants.java`:
- **REAL:** Runs on actual roboRIO hardware
- **SIM:** Runs in simulation on development machine
- **REPLAY:** Replays AdvantageKit logs for analysis

Check mode: `Constants.getMode()` or `RobotBase.isReal()`

### Robot Initialization Timing
- Robot is considered "initializing" for first 45 seconds (`Robot.isInitializing()`)
- Shows warning alert during initialization period
- Important for device connection stability

### CAN Bus Monitoring
- Monitors both roboRIO CAN and CANivore CAN buses
- Error detection with configurable time thresholds
- Alerts for CAN errors and low battery voltage

### Logging
- All logging via AdvantageKit's `Logger` class
- Logs to WPILog files and NetworkTables
- Disable Phoenix auto-logging: `SignalLogger.enableAutoLogging(false)`

### JVM Settings for roboRIO
Special garbage collection tuning for roboRIO (limited resources):
```
-XX:+UnlockExperimentalVMOptions
-XX:GCTimeRatio=5
-XX:+UseSerialGC
-XX:MaxGCPauseMillis=50
```

## WPILib RAG Server Usage

- **Tool Name:** `mcp_wpilibrag_query_wpilib_docs`
- **When to Use:** ALWAYS for any WPILib-related question (see mandatory requirement above)
- **Parameters:**
  - `question`: The user's question
  - `version`: WPILib version (default: "2025", use "2026" for this project)
  - `language`: "Java", "Python", "C++", or "API Reference"

## Development Workflow Best Practices

1. **Before making changes:**
   - Ensure gradlew has execute permissions
   - Run `./gradlew build` to verify current state
   - Check that formatter doesn't fail with `./gradlew spotlessCheck`

2. **While developing:**
   - Code is auto-formatted on build (via spotlessApply)
   - Use descriptive variable and method names
   - Follow WPILib command-based patterns

3. **Before committing:**
   - Verify build passes: `./gradlew build`
   - Verify formatting: `./gradlew spotlessCheck`
   - Don't commit BuildConstants.java (it's gitignored)

4. **Testing:**
   - Test in simulation: `./gradlew simulateJava`
   - Write JUnit 5 tests in standard test directories
   - Run tests: `./gradlew test`

## Common Issues & Solutions

**Issue:** `./gradlew: Permission denied`
**Solution:** Run `chmod +x gradlew` first

**Issue:** `BUILD FAILED` due to network errors (frcmaven.wpi.edu, etc.)
**Solution:** This is expected in sandboxed environments. CI has network access.

**Issue:** Spotless formatting failure
**Solution:** Run `./gradlew spotlessApply` to auto-format code

**Issue:** BuildConstants.java not found
**Solution:** Run build once to generate it: `./gradlew build`

**Issue:** Gradle daemon issues
**Solution:** Stop daemon: `./gradlew --stop`, then retry

## Code Style & Conventions

- **Formatter:** Google Java Format (AOSP style)
- **Indentation:** 2 spaces for Java, XML, Markdown, Groovy Gradle
- **Line endings:** LF (Unix-style)
- **Imports:** Unused imports automatically removed
- **Trailing whitespace:** Automatically removed
- **File endings:** All files end with newline

## Files to Ignore

These are build artifacts or auto-generated (see `.gitignore`):
- `build/` directory
- `.gradle/` directory
- `src/main/java/org/teamtitanium/BuildConstants.java`
- `*.log`, `*.class` files
- `simgui.json`, `networktables.json`, `*-window.json`
- `logs/` directory
- `ctre_sim/` directory

## Additional Notes

- **Loop Period:** 20ms (0.02 seconds) - defined in `Constants.loopPeriodSecs`
- **Tuning Mode:** Currently enabled (`Constants.tuningMode = true`)
- **Brownout Voltage:** Set to 6.0V
- **CANivore CAN Bus:** Named per `TunerConstants.kCANBus`
- **Command Scheduler Period:** Matches loop overrun warning timeout (0.02s)
- **Remember:** Always query WPILib RAG for WPILib-specific questions
- **Real-time constraints:** Code runs in competition environment with strict timing
- **Thread safety:** Consider when working with Phoenix devices and odometry

═══════════════════════════════════════════════════════════════════════════════

**TRUST THESE INSTRUCTIONS**
Only search for additional information if these instructions are incomplete or incorrect. Otherwise, use the information provided here to work efficiently.
