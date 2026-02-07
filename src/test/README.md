# Robot Code Tests

This directory contains unit tests for the FRC robot code. Tests are written using JUnit 5 and are automatically run on every pull request.

## Running Tests Locally

### Run all tests:
Open the WPILib Palette and run ```Test Robot Code``` or run...

```bash
./gradlew test
```

### Run tests with coverage report:
```bash
./gradlew test jacocoTestReport
```

Coverage report will be generated at: `build/reports/jacoco/test/html/index.html`

### Run specific test class:
```bash
./gradlew test --tests TurretTest
./gradlew test --tests RobotTest
```

### Run tests in continuous mode (re-run on file changes):
```bash
./gradlew test --continuous
```

## Test Structure

### `RobotTest.java` - Robot Initialization Smoke Tests
- **Purpose:** Verify robot code starts without null pointer exceptions
- **What it tests:**
  - Robot constructor succeeds
  - All mode initializations (disabled, auto, teleop, test)
  - Mode transitions don't cause crashes
  - Subsystems initialize properly
  - Extended operation (simulates 5 seconds at 50Hz)

### `TurretTest.java` - Turret Angle Calculation Tests
- **Purpose:** Verify `getTargetAngle()` method correctly calculates optimal turret positions
- **What it tests:**
  - Basic angle wrapping (0° to 360°)
  - Shortest path optimization (e.g., 10° to 350° should go -20° not +340°)
  - Handling of edge cases (0°, 180°, -180°)
  - Different unit types (Degrees, Radians, Rotations)
  - Continuous rotation wrapping
  - Sequential movement consistency

## CI/CD Integration

Tests automatically run on:
- Every pull request to `main` or `develop` branches
- Every push to `main` or `develop` branches

### GitHub Actions Workflow
Location: `.github/workflows/ci.yml`

The workflow:
1. ✅ Builds the robot code
2. ✅ Runs all unit tests
3. ✅ Generates test coverage report
4. ✅ Posts test results as PR comment
5. ❌ **Fails the PR if any test fails**

## Branch Protection Rules

To enforce tests on pull requests:

1. Go to your repository on GitHub
2. Navigate to **Settings** → **Branches**
3. Add rule for `main` branch:
   - ✅ Require status checks to pass before merging
   - ✅ Select "CI - Build and Test" workflow
   - ✅ Require branches to be up to date before merging

This ensures no PR can be merged if tests are failing!

## Writing New Tests

### Test Naming Convention
- Test files: `*Test.java`
- Test methods: `test*()` or use `@DisplayName` annotation
- Use descriptive names that explain what is being tested

### Example Test Structure
```java
@Test
@DisplayName("Should handle edge case correctly")
void testEdgeCase() {
    // Arrange
    Turret turret = new Turret(new TurretIO() {});

    // Act
    Angle result = turret.getTargetAngle(target, current);

    // Assert
    assertEquals(expected, result, tolerance, "Failure message");
}
```

### Parameterized Tests
Use `@ParameterizedTest` for testing multiple scenarios:
```java
@ParameterizedTest
@CsvSource({
    "0, 90, 90",
    "90, -90, -90",
    "350, 10, 10"
})
void testMultipleScenarios(double current, double target, double expected) {
    // Test logic
}
```

## Test Coverage

Current coverage target: **30% minimum**

View coverage report after running tests:
```bash
./gradlew test jacocoTestReport
open build/reports/jacoco/test/html/index.html  # macOS
start build/reports/jacoco/test/html/index.html  # Windows
```

## Debugging Failed Tests

### View test results:
```bash
# Results summary
cat build/test-results/test/*.xml

# Detailed HTML report
open build/reports/tests/test/index.html
```

### Common test failures:

1. **HAL initialization errors**
   - Solution: Ensure `HAL.initialize()` is called in `@BeforeEach`

2. **Null pointer exceptions**
   - Solution: Check that all subsystems have mock IOs in test setup

3. **Timeout errors**
   - Solution: Add `@Timeout(seconds)` annotation or increase timeout

4. **Flaky tests (pass sometimes, fail others)**
   - Solution: Look for race conditions or uninitialized state

## Best Practices

✅ **DO:**
- Test edge cases and boundary conditions
- Use descriptive test names with `@DisplayName`
- Clean up resources in `@AfterEach`
- Use `assertDoesNotThrow()` for smoke tests
- Test one thing per test method

❌ **DON'T:**
- Test implementation details (test behavior, not internals)
- Write tests that depend on execution order
- Use real hardware in unit tests (use mocks/simulations)
- Ignore failing tests (fix them or remove them!)

## Continuous Improvement

As the robot code grows, continuously:
- Add tests for new features
- Improve coverage of critical systems
- Refactor tests to reduce duplication
- Update this README with new testing patterns

## Questions?

Contact the programming leads or check:
- [WPILib Unit Testing Docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html)
- [JUnit 5 User Guide](https://junit.org/junit5/docs/current/user-guide/)
