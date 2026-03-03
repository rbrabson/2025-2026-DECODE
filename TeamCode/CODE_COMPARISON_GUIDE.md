# TaigaBots 18190 -- Code Modularization Guide
## Old Code vs. New Code: A Block-by-Block Comparison

This document walks through every major block of the **original JWBF.java** (the
Blue Far Autonomous, 1073 lines), explains what each block does, shows how it was
refactored into the new modular architecture, and explains *why* each change was
made. The same patterns apply to JWRC, JWBC, JWRF, JWBF_Finish, jwb, and jwr.

> **How to use this guide**: Read each section, look at the "Old Code" snippet,
> then look at the "New Code" and the explanation. Try to understand *why* the
> change makes the code better. Discuss with your teammates!

---

## Table of Contents

1. [Package & Imports (Lines 1-43)](#1-package--imports-lines-1-43)
2. [Class Declaration & Annotations (Lines 44-45)](#2-class-declaration--annotations-lines-44-45)
3. [Variable Declarations (Lines 46-127)](#3-variable-declarations-lines-46-127)
4. [The count() Utility Method (Lines 130-137)](#4-the-count-utility-method-lines-130-137)
5. [The runIntake() Method (Lines 138-144)](#5-the-runintake-method-lines-138-144)
6. [The turretTracker() Method (Lines 148-217)](#6-the-turrettracker-method-lines-148-217)
7. [The automated_shoot() Method (Lines 224-564)](#7-the-automated_shoot-method-lines-224-564)
8. [Pose Definitions (Lines 569-610)](#8-pose-definitions-lines-569-610)
9. [Path Building - buildPaths() (Lines 614-676)](#9-path-building---buildpaths-lines-614-676)
10. [Path State Machine - autonomousPathUpdate() (Lines 682-828)](#10-path-state-machine---autonomouspathupdate-lines-682-828)
11. [The setPathState() Helper (Lines 832-835)](#11-the-setpathstate-helper-lines-832-835)
12. [The Main loop() Method (Lines 840-935)](#12-the-main-loop-method-lines-840-935)
13. [The init() Method (Lines 940-1046)](#13-the-init-method-lines-940-1046)
14. [Lifecycle Methods: start() and stop() (Lines 1057-1067)](#14-lifecycle-methods-start-and-stop-lines-1057-1067)
15. [Summary of All Changes](#15-summary-of-all-changes)
16. [Discussion Questions for the Team](#16-discussion-questions-for-the-team)

---

## 1. Package & Imports (Lines 1-43)

### What the old code does

The old file lives in a single flat package with ALL other files:

```java
package org.firstinspires.ftc.teamcode.pedroPathing;
```

It then has 25+ import statements pulling in everything from PedroPathing,
Limelight, Rev hardware, servos, motors, vision, and timers:

```java
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
// ... and 20+ more imports
```

Every single file (JWBF, JWRC, JWBC, jwb, jwr, etc.) had the **exact same**
set of imports copy-pasted at the top.

### How it was modified

The new code is split across multiple packages:

```
org.firstinspires.ftc.teamcode.
    util/                  -- PatternUtil, AllianceConfig, SharedState
    hardware/              -- RobotHardware, IndexerConfig
    subsystems/            -- TurretSubsystem, ShootingStateMachine, etc.
    autonomous/            -- BaseAutonomous, BlueFarAuto, etc.
    teleop/                -- MainTeleOp, TeleOpBlue, TeleOpRed
    test/                  -- TestMenuOpMode, DriveMotorTest, etc.
```

Each file now **only imports what it actually needs**. For example,
`BlueFarAuto.java` imports path-building classes and its base class -- it does
NOT import Limelight, color sensors, or motor classes because those are handled
by `BaseAutonomous` and the subsystem classes.

### Why

| Problem with the old way | How the new way fixes it |
|---|---|
| Everything in one folder = hard to find anything | Organized by *purpose* -- subsystems, hardware, autonomous, etc. |
| 25+ identical import blocks in every file | Each file imports only what it needs |
| Changing one import requires editing 9+ files | Shared imports live in base classes |
| Newcomers can't tell what a file is responsible for | Package names tell you immediately |

---

## 2. Class Declaration & Annotations (Lines 44-45)

### What the old code does

```java
@Autonomous(name="JWBF")
public class JWBF extends OpMode {
```

- `@Autonomous(name="JWBF")` -- Registers this as an autonomous OpMode visible
  on the Driver Station. The name "JWBF" appears in the autonomous selection menu.
- `extends OpMode` -- Uses the iterative OpMode style (init/loop pattern), which
  is the correct choice for PedroPathing since Pedro needs continuous loop updates.

### How it was modified

```java
// In BlueFarAuto.java:
@Autonomous(name = "Blue Far")
public class BlueFarAuto extends BaseAutonomous {
```

```java
// In BaseAutonomous.java:
public abstract class BaseAutonomous extends OpMode {
```

### Why

| Aspect | Old | New | Reason |
|---|---|---|---|
| **Name** | "JWBF" | "Blue Far" | Descriptive name tells drivers which auto to select |
| **Extends** | `OpMode` directly | `BaseAutonomous` (which extends `OpMode`) | All shared loop/init logic lives in the base class |
| **Class name** | `JWBF` | `BlueFarAuto` | Name describes what it IS, not initials |
| **Abstract base** | None -- everything in one class | `BaseAutonomous` is abstract | Forces every auto to define start pose, alliance, paths, etc. |

**Key concept -- Inheritance**:
`BaseAutonomous` contains ~200 lines of shared logic (hardware init, loop
orchestration, telemetry, subsystem wiring). Each concrete autonomous
(`BlueFarAuto`, `RedCenterAuto`, etc.) only contains the ~100-150 lines that
are *unique* to that specific routine (poses, paths, state machine sequence).

---

## 3. Variable Declarations (Lines 46-127)

### What the old code does

This is the largest block of declarations -- approximately 80 lines defining:

**Alliance-specific constants (hard-coded per file):**
```java
private final double Bx = 0;          // Blue basket X coordinate
private final double By = 144;        // Blue basket Y coordinate
private final double m = ((double) 742 / (double) 90);  // Turret ticks per degree
```

**Motor and servo references:**
```java
private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, turret;
private DcMotorEx shooter1;
private Servo hoodExtension, indexer, hinge;
private DcMotor intake;
```

**Indexer positions (hard-coded per file, DIFFERENT between JWBF and JWRC!):**
```java
private double increment = 0.2084;
private double pos1Intake = 0.3935;
private double pos2Intake = 0.596;
private double pos3Intake = 0.8045;
private double pos1Shoot = 0.0829;
private double pos2Shoot = 0.2856;
private double pos3Shoot = 0.494;
```

**Turret limits:**
```java
private int turretExtremeLeft = 1700;
private int turretExtremeRight = -350;
```

**Game state variables:**
```java
private String motif = "";
private String pattern = "GPP";
private boolean shooting = false;
private boolean shooting2 = false;
private int iteration = 0;
private boolean stopShooting = true;
private boolean flag = true;
// ... and 15+ more
```

**ElapsedTime timers (18 of them!):**
```java
ElapsedTime rightTriggerDuration = new ElapsedTime();
ElapsedTime intakeDelay = new ElapsedTime();
ElapsedTime rightBumperDuration = new ElapsedTime();
ElapsedTime indexerTime = new ElapsedTime();
ElapsedTime hingeTime = new ElapsedTime();
ElapsedTime hinge12 = new ElapsedTime();
// ... and 12 more
```

### How it was modified

These 80+ variables were distributed to the classes that actually USE them:

| Old Variable(s) | New Location | Why it moved there |
|---|---|---|
| `Bx`, `By` | `AllianceConfig` enum (BLUE=0,144 / RED=144,144) | Alliance-specific, shared by all autos |
| `m` (ticks per degree) | `RobotHardware.TURRET_TICKS_PER_DEGREE` | Hardware constant, belongs with hardware |
| `turretExtremeLeft/Right` | `RobotHardware.TURRET_EXTREME_LEFT/RIGHT` | Physical limits of the turret motor |
| All motor/servo references | `RobotHardware` class fields | Hardware init belongs in one central place |
| `pos1Intake`, `pos2Shoot`, etc. | `IndexerConfig` (CONFIG_A and CONFIG_B) | Two different configs for two robots |
| `increment` | `IndexerConfig.increment` | Goes with other indexer positions |
| `motif`, `detection` | `TurretSubsystem` fields | Part of turret/vision tracking |
| `pattern`, `indexerState`, `centerControl` | `IndexerSubsystem` fields | Part of indexer management |
| `shooting`, `iteration`, `flag`, `stopShooting` | `ShootingStateMachine` fields | Part of the shooting logic |
| 3 shooting ElapsedTimes | Replaced by single `timer` in `ShootingStateMachine` | State machine needs only ONE timer |
| 15+ other ElapsedTimes | Eliminated or moved to specific subsystems | Many were unused or redundant |

### Why

| Problem | Solution |
|---|---|
| 80+ variables in one class = impossible to understand | Each class has only its own 5-10 relevant variables |
| `Bx`/`By` hard-coded differently in JWBF vs JWRC | `AllianceConfig` enum holds both, select with BLUE or RED |
| Indexer positions different in JWBF vs JWRC (CONFIG_A vs CONFIG_B) | `IndexerConfig` class with two static instances |
| 18 ElapsedTime timers, hard to know which does what | Most eliminated; shooting uses ONE timer with state-based logic |
| Changing a servo name means editing 9+ files | Change it ONCE in `RobotHardware` |

**Key concept -- Single Source of Truth**:
Every piece of configuration data now lives in exactly ONE place. If you need to
change the turret's encoder limit from 1700 to 1800, you change ONE line in
`RobotHardware.java`, not nine.

---

## 4. The count() Utility Method (Lines 130-137)

### What the old code does

```java
public static int count(String str, Character targetChar) {
    int iter = 0;
    for (int i = 0; i < str.length(); i++) {
        if (str.charAt(i) == targetChar) {
            iter++;
        }
    }
    return iter;
}
```

This counts how many times a character appears in a string. For example,
`count("GPP", 'P')` returns 2. It's used to check how many green vs purple
balls are in the indexer pattern.

**The problem**: This exact method was copy-pasted into ALL 9 files (5
autonomous + 2 teleop + 2 others). Any bug fix required editing 9 copies.

### How it was modified

```java
// In util/PatternUtil.java:
public static int count(String str, Character targetChar) {
    // Exact same logic, now in ONE place
}

// Plus new helper methods that were needed:
public static String replaceAt(String pattern, int index, char newChar) { ... }
public static boolean hasEmptySlots(String pattern) { ... }
public static int getNextEmptySlot(String pattern) { ... }
```

All 9 files now call `PatternUtil.count(...)` instead of their own copy.

### Why

This is the textbook example of the **DRY Principle** (Don't Repeat Yourself):
- **Old**: 9 identical copies. Bug in one? You have to find and fix all 9.
- **New**: 1 copy in `PatternUtil`. Fix it once, it's fixed everywhere.

We also added `replaceAt()` because the old code used an inline string
manipulation expression that was repeated in many places:
```java
// Old (repeated in loop() and other methods):
pattern = pattern.substring(0, indexerState) + "G" + pattern.substring(indexerState + 1);

// New:
pattern = PatternUtil.replaceAt(pattern, currentEmptySlot, 'G');
```

---

## 5. The runIntake() Method (Lines 138-144)

### What the old code does

```java
public void runIntake(boolean bool) {
    if (bool) {
        intake.setPower(1);
    } else {
        intake.setPower(0);
    }
}
```

A simple on/off toggle for the intake motor. Called with `runIntake(true)` in the
main loop to keep the intake spinning during autonomous.

### How it was modified

```java
// In subsystems/IntakeSubsystem.java:
public class IntakeSubsystem {
    private final DcMotor intakeMotor;

    public IntakeSubsystem(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    public void run(boolean on) {
        intakeMotor.setPower(on ? 1.0 : 0.0);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}
```

### Why

While the original method was simple, wrapping it in its own class provides:

1. **Encapsulation**: The intake motor reference is private. Nobody can
   accidentally set the wrong motor power elsewhere in the code.
2. **Extensibility**: If we later want to add intake current monitoring (to
   detect jams), or variable speed control, we add it in ONE place.
3. **Consistency**: Every mechanism is a subsystem with the same pattern:
   constructor takes hardware, methods control the mechanism.

---

## 6. The turretTracker() Method (Lines 148-217)

### What the old code does

This 70-line method does THREE things:

**Part A -- Calculate turret angle based on robot position (lines 149-170):**
```java
if (detection) {
    // After motif detected: aim at the blue basket (Bx=0, By=144)
    double targetAngleDeg = 180 - Math.toDegrees(
        Math.atan((By - follower.getPose().getY()) / (follower.getPose().getX() - Bx)));
    double robotHeadingDeg = Math.toDegrees(follower.getHeading());
    double turretAngleDeg = targetAngleDeg - (robotHeadingDeg - 90);
    turretPose = (int) (turretAngleDeg * m);
    if (turretPose > turretExtremeLeft || turretPose < turretExtremeRight) return;
} else {
    // Before detection: aim at field center (72, 144) as fallback
    double targetAngleDeg = (Math.toDegrees(
        Math.atan((144 - follower.getPose().getY()) / (72 - follower.getPose().getX())))) % 180;
    // ... same pattern
}
```

**CRITICAL DIFFERENCE between files**: JWBF uses `180 - atan(...)` while JWRC
uses `((atan(...) % 180) + 180) % 180`. These are different formulas for the
same purpose because the target basket is in a different corner of the field!

**Part B -- Read Limelight data and detect motif (lines 172-207):**
```java
LLResult result1 = limelight.getLatestResult();
if (result1 != null && result1.isValid()) {
    double error = result1.getTx();
    if (Math.abs(error) < deadband) {
        gamepad1.rumble(100);
        gamepad2.rumble(100);
    }
    if (!detection) {
        List<LLResultTypes.FiducialResult> fiducials = result1.getFiducialResults();
        if (!fiducials.isEmpty()) {
            int tagId = fiducials.get(0).getFiducialId();
            if (tagId == 21) motif = "GPP";
            else if (tagId == 22) motif = "PGP";
            else if (tagId == 23) motif = "PPG";
            limelight.pipelineSwitch(1);
            detection = true;
        }
    }
}
```

**Part C -- Move turret motor (line 210):**
```java
turret.setTargetPosition(turretPose);
```

### How it was modified

All of this is now in `subsystems/TurretSubsystem.java`:

```java
public class TurretSubsystem {
    private final DcMotor turretMotor;
    private final Limelight3A limelight;
    private final double targetX;    // From AllianceConfig (0 for Blue, 144 for Red)
    private final double targetY;    // From AllianceConfig (always 144)

    public void update(double robotX, double robotY, double robotHeading,
                       Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        // Step 1: Calculate angle to target
        double targetAngleDeg;
        if (motifDetected) {
            targetAngleDeg = calculateAngleToTarget(robotX, robotY, targetX, targetY);
        } else {
            targetAngleDeg = calculateAngleToTarget(robotX, robotY, 72, 144);
        }
        // Step 2: Convert to turret-relative angle
        double robotHeadingDeg = Math.toDegrees(robotHeading);
        double turretAngleDeg = targetAngleDeg - (robotHeadingDeg - 90);
        turretPose = (int) (turretAngleDeg * M);
        // Step 3: Safety check
        if (turretPose > EXTREME_LEFT || turretPose < EXTREME_RIGHT) return;
        // Step 4: Process Limelight and detect motif
        // Step 5: Move turret
        turretMotor.setTargetPosition(turretPose);
    }

    private double calculateAngleToTarget(double robotX, double robotY,
                                           double targetX, double targetY) {
        double rawAngle = Math.toDegrees(
            Math.atan((targetY - robotY) / (targetX - robotX)));
        return ((rawAngle % 180) + 180) % 180;
    }
}
```

### Key changes and why

| Change | Old | New | Why |
|---|---|---|---|
| **Angle formula** | Two different formulas in JWBF vs JWRC | One unified formula with `targetX`/`targetY` params | Eliminates error-prone copy-paste; math works for both alliances |
| **Target coordinates** | Hard-coded `Bx=0, By=144` in JWBF; `144, 144` in JWRC | Passed via constructor from `AllianceConfig` | One class works for both alliances |
| **Motif detection** | Inline in turretTracker(), mixed with angle math | Separate `detectMotifFromAprilTag()` private method | Each method does ONE thing (Single Responsibility) |
| **State variables** | `detection`, `motif` floating in the OpMode class | Private fields of `TurretSubsystem` | Encapsulated -- nobody can accidentally change detection state |
| **Robot position** | Directly accesses `follower.getPose()` | Received as parameters (`robotX`, `robotY`, `robotHeading`) | Decoupled from follower -- subsystem doesn't need to know about PedroPathing |

**The angle formula unification is important**: The old JWBF used
`180 - atan(dy/dx)` while JWRC used `((atan(dy/dx) % 180) + 180) % 180`.
These look very different but both were trying to normalize the atan result to
a usable range. The new code uses the modular formula
`((rawAngle % 180) + 180) % 180` which works correctly for ALL quadrants on the
field, regardless of which corner the target basket is in. The difference in
behavior between Blue and Red alliances is handled entirely by passing different
`targetX` / `targetY` values.

---

## 7. The automated_shoot() Method (Lines 224-564)

### What the old code does

This is the **largest and most complex block** in the entire codebase -- **340
lines** of deeply nested if/else logic. It controls the sequence of shooting all
3 balls from the indexer.

**The high-level flow:**

1. Check if `launch` is true (shooting requested).
2. Compare the `motif` (from AprilTag) with the `pattern` (balls in indexer)
   to determine which slot to shoot first.
3. For each of the 3 balls (`iteration` 0, 1, 2):
   a. Move the indexer servo to the correct shoot position
   b. Wait for `indexerTime` milliseconds
   c. Open the hinge (set to 0.4)
   d. Wait for `hingeTime` milliseconds
   e. Close the hinge (set to 0.09)
   f. Wait for `hinge12` milliseconds
   g. Increment `iteration`

**The nesting structure:**

```
if (launch) {
    if (count(green)==1 && count(purple)==2 && !motif.isEmpty()) {
        // CASE: Valid motif detected
        if (motifDetect == patternDetect) {
            // Sub-case: Green positions match
            if (iteration == 0) {
                if (!centerControl) {
                    // Move indexer, wait, open hinge, wait, close hinge, wait
                } else {
                    // Center control: skip indexer move
                }
            }
            if (iteration == 1) { /* same pattern, different position */ }
            if (iteration == 2) { /* same pattern, different position, set stopShooting=true */ }
        } else if (motifDetect == (patternDetect+1)%3) {
            // Sub-case: Offset by +1
            if (iteration == 0) { /* SAME 20 LINES AGAIN with different servo position */ }
            if (iteration == 1) { /* SAME 15 LINES AGAIN */ }
            if (iteration == 2) { /* SAME 15 LINES AGAIN */ }
        } else {
            // Sub-case: Offset by +2
            if (iteration == 0) { /* SAME 20 LINES AGAIN with different servo position */ }
            if (iteration == 1) { /* SAME 15 LINES AGAIN */ }
            if (iteration == 2) { /* SAME 15 LINES AGAIN */ }
        }
    } else {
        // CASE: No motif -- shoot in default order
        if (iteration == 0) { /* SAME 20 LINES AGAIN */ }
        if (iteration == 1) { /* SAME 15 LINES AGAIN */ }
        if (iteration == 2) { /* SAME 15 LINES AGAIN */ }
    }
} else {
    // Reset everything
}
```

**Let's count the repetition**:
- The "move indexer, wait, open hinge, wait, close hinge, wait" pattern is
  written **12 times** (3 iterations x 4 cases).
- Each repetition is 15-20 lines.
- The ONLY differences between repetitions are:
  - Which servo position to use (`pos1Shoot`, `pos2Shoot`, `pos3Shoot`, or
    `posNShoot + increment`)
  - The timing delays (first shot: 500/675/850ms; subsequent: 150/325/500ms;
    center control: 175/350ms)
  - Whether `stopShooting = true` on the last iteration

### How it was modified

**The entire 340-line method was replaced by `ShootingStateMachine.java`** (about
140 lines of actual logic), using an enum-based state machine:

```java
public enum State {
    IDLE,           // Not shooting
    MOVE_INDEXER,   // Position the indexer servo
    WAIT_INDEXER,   // Wait for servo to arrive
    WAIT_HINGE,     // Wait for hinge to open (ball launches)
    WAIT_CLOSE,     // Wait for hinge to close
    FINISHED        // All 3 done, clean up
}
```

The update() method is a clean switch statement:

```java
public void update(Telemetry telemetry) {
    if (!shooting) return;

    switch (state) {
        case MOVE_INDEXER:
            indexer.setPosition(shootOrder[currentIteration]);
            timer.reset();
            state = State.WAIT_INDEXER;
            break;

        case WAIT_INDEXER:
            double delay = isFirstShot ? firstShotIndexerDelay : SUBSEQUENT_INDEXER_DELAY;
            if (timer.milliseconds() > delay) {
                hinge.setPosition(HINGE_OPEN);
                timer.reset();
                state = State.WAIT_HINGE;
            }
            break;

        case WAIT_HINGE:
            if (timer.milliseconds() > hingeDelay) {
                hinge.setPosition(HINGE_CLOSED);
                timer.reset();
                state = State.WAIT_CLOSE;
            }
            break;

        case WAIT_CLOSE:
            if (timer.milliseconds() > closeDelay) {
                currentIteration++;
                if (currentIteration >= 3) {
                    state = State.FINISHED;
                } else {
                    state = State.MOVE_INDEXER;
                }
            }
            break;

        case FINISHED:
            shooting = false;
            pattern = "XXX";
            state = State.IDLE;
            break;
    }
}
```

**The motif alignment logic** was extracted to `calculateShootOrder()`:

```java
private double[] calculateShootOrder(String pattern, String motif) {
    int motifGreenPos = motif.indexOf('G');
    int patternGreenPos = pattern.indexOf('G');

    if (motifGreenPos == patternGreenPos) {
        return new double[]{ config.pos1Shoot, config.pos2Shoot, config.pos3Shoot };
    } else if (motifGreenPos == (patternGreenPos + 1) % 3) {
        return new double[]{ config.pos3Shoot, config.pos3Shoot + config.increment, ... };
    } else {
        return new double[]{ config.pos2Shoot, config.pos2Shoot + config.increment, ... };
    }
}
```

### Why (this is the most important refactor in the entire project!)

| Problem | How the state machine solves it |
|---|---|
| **340 lines** of nested if/else | **~80 lines** of clean switch/case |
| Same 15-line block repeated **12 times** | Written **once**, parameterized by `shootOrder[]` and timing constants |
| 3 separate timers (`indexerTime`, `hingeTime`, `hinge12`) | **1 timer**, reset at each state transition |
| `flag` boolean toggled back and forth | Eliminated -- state transitions handle this naturally |
| `iteration` counter with manual reset | `currentIteration` with automatic increment |
| `stopShooting` boolean checked in multiple places | State machine reaches `FINISHED` state, resets cleanly |
| Different timing for first shot vs subsequent shots | Clean if/else on `isFirstShot` in each state |
| Hard to trace what happens when | Each state has ONE clear purpose and ONE clear transition |
| Different servo positions hard-coded inline | Pre-calculated `shootOrder[]` array |
| Bug in one iteration won't be caught in others | One code path for ALL iterations |

**Key concept -- State Machines**:
A state machine is a programming pattern where:
1. You define a set of named **states** (IDLE, MOVE_INDEXER, WAIT_HINGE, etc.)
2. At any given moment, the system is in exactly **one** state
3. Each state has clear **transitions** to other states
4. You can draw a diagram showing all states and transitions

Compare this to the old approach where the system's "state" was spread across
`iteration`, `flag`, `stopShooting`, three timers, and the nesting level of the
if/else tree. Debugging meant mentally tracking all those variables simultaneously.

---

## 8. Pose Definitions (Lines 569-610)

### What the old code does

```java
private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
private final Pose ShootPose = new Pose(47.845..., 95.668..., Math.toRadians(134));
private final Pose ShootPose1 = new Pose(58.5, 13.723..., Math.toRadians(118));
private final Pose pickup1Pose = new Pose(44.574..., 83.077..., Math.toRadians(180));
// ... 15+ more pose definitions
private final Pose Ctrl1 = new Pose(55.969..., 48.856...);  // Control point (no heading)
private final Pose Ctrl2 = new Pose(64.588..., 51.936...);
// ... more control points
```

These define every waypoint the robot visits during autonomous. Each Pose has an
(x, y) coordinate in inches and a heading in radians. Control points (Ctrl1, Ctrl2, etc.)
are used for BezierCurve paths and don't need headings.

### How it was modified

The poses **stayed in the concrete autonomous class** (e.g., `BlueFarAuto.java`),
not in the base class:

```java
// In BlueFarAuto.java (same poses, same values):
private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
private final Pose ShootPose = new Pose(47.845..., 95.668..., Math.toRadians(134));
// ... etc.
```

### Why -- minimal change here

Poses are the one thing that is truly **unique to each autonomous routine**.
Blue Far has different poses than Red Center. So the poses correctly belong in
each concrete class. We did NOT try to share them through the base class.

What DID change:
- The base class provides `getStartPose()` as an abstract method, so it can set
  the follower's starting pose in `init()` without knowing the specific values.
- Pose values are now next to the `buildPaths()` method that uses them, making
  the connection between waypoints and paths easier to see.

---

## 9. Path Building -- buildPaths() (Lines 614-676)

### What the old code does

```java
public void buildPaths() {
    scorePreload = follower.pathBuilder()
        .addPath(new BezierLine(startPose, ShootPose1))
        .setLinearHeadingInterpolation(startPose.getHeading(), ShootPose1.getHeading())
        .build();

    grabPickup4 = follower.pathBuilder()
        .addPath(new BezierLine(ShootPose1, pickup4Pose))
        .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup4Pose.getHeading())
        .setBrakingStrength(1)
        .build();
    // ... 13 more path chains
}
```

This builds all the PathChain objects that define the robot's routes. Each path
uses either a `BezierLine` (straight line) or `BezierCurve` (curved path through
control points). The heading interpolation controls which direction the robot
faces while traveling.

### How it was modified

The method signature changed to accept a `Follower` parameter (provided by the
base class), and it's now an `@Override` of the abstract method:

```java
// In BlueFarAuto.java:
@Override
protected void buildPaths(Follower follower) {
    scorePreload = follower.pathBuilder()
        .addPath(new BezierLine(startPose, ShootPose1))
        .setLinearHeadingInterpolation(startPose.getHeading(), ShootPose1.getHeading())
        .build();
    // ... same path definitions
}
```

```java
// In BaseAutonomous.java init():
buildPaths(follower);  // Calls the subclass implementation
```

### Why

| Change | Reason |
|---|---|
| Method receives `follower` as parameter instead of accessing a field | The follower is created and owned by the base class; the subclass doesn't need its own reference |
| `@Override` annotation | Compiler verifies the method signature matches the base class definition |
| Called from base class `init()` | Ensures paths are always built after follower is created, in the correct order |

The actual path definitions are unchanged because they're unique to each
autonomous routine.

---

## 10. Path State Machine -- autonomousPathUpdate() (Lines 682-828)

### What the old code does

A large switch statement with 20 cases (0 through 19) that sequences the
autonomous routine:

```java
public void autonomousPathUpdate() {
    switch (pathState) {
        case 0:
            follower.followPath(scorePreload);
            setPathState(1);
            break;
        case 1:
            if (!follower.isBusy()) {
                if (flag1) {
                    shooting = true;    // <-- Sets the shooting flag directly
                    flag1 = false;
                }
                setPathState(2);
            }
            break;
        case 2:
            if (!shooting) {            // <-- Checks the shooting flag directly
                follower.followPath(grabPickup4);
                setPathState(3);
            }
            break;
        // ... 17 more cases following the same pattern
    }
}
```

The pattern repeats 5 times (once per scoring cycle):
- **Start path** -> **Wait for arrival + trigger shooting** -> **Wait for shooting to finish** -> **Start next path**

Helper flags (`flag1` through `flag5`) ensure `shooting = true` is set only once
per cycle, not every loop iteration.

### How it was modified

```java
// In BlueFarAuto.java:
@Override
protected void autonomousPathUpdate() {
    switch (pathState) {
        case 0:
            follower.followPath(scorePreload);
            setPathState(1);
            break;
        case 1:
            if (!follower.isBusy()) {
                if (flag1) {
                    triggerShoot();     // <-- Calls base class helper method
                    flag1 = false;
                }
                setPathState(2);
            }
            break;
        case 2:
            if (!shooter.isShooting()) {  // <-- Queries the state machine object
                follower.followPath(grabPickup4);
                setPathState(3);
            }
            break;
        // ... same structure
    }
}
```

### Key changes

| Old | New | Why |
|---|---|---|
| `shooting = true` | `triggerShoot()` | Base class method that properly initializes the ShootingStateMachine with pattern, motif, and centerControl state |
| `!shooting` | `!shooter.isShooting()` | Queries the actual state machine instead of a loose boolean |
| `automated_shoot(shooting)` called in `loop()` | `shooter.update(telemetry)` called in base class `loop()` | The shooting logic is self-contained in its own class |
| Path state machine + shooting logic in same class | Path state machine in subclass, shooting logic in `ShootingStateMachine` | Separation of concerns: "where to go" vs "how to shoot" |

The path state machine itself (the 20 cases) is essentially unchanged because
it IS unique to each autonomous. What changed is HOW it interacts with shooting.

---

## 11. The setPathState() Helper (Lines 832-835)

### What the old code does

```java
public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
}
```

A simple helper that changes the current path state and resets the timer.

### How it was modified

```java
// In BaseAutonomous.java:
protected void setPathState(int newState) {
    pathState = newState;
    pathTimer.resetTimer();
}
```

### Why

Moved to the base class with `protected` access so all subclasses can use it.
The logic is identical -- it was already clean.

---

## 12. The Main loop() Method (Lines 840-935)

### What the old code does

This is the OpMode's main loop, called repeatedly after START. It orchestrates
everything:

```java
@Override
public void loop() {
    char green = 'G';
    char purple = 'P';
    char x1 = 'X';

    // Set hood position
    hoodExtension.setPosition(0);

    // Update path following
    follower.update();
    autonomousPathUpdate();

    // Update turret, shooter, intake
    turretTracker(true);
    shooter1.setVelocity(1120);
    automated_shoot(shooting);
    runIntake(true);

    // CENTER CONTROL: Pre-position indexer when all balls loaded
    if ((count(pattern, x1) == 0) && !centerControl && !motif.isEmpty()) {
        // ... 15 lines of motif alignment logic to determine which
        // shoot position to pre-move the indexer to
        centerControl = true;
    }

    // INTAKE POSITIONING: Move indexer to next empty slot
    indexerState = pattern.indexOf("X");
    if (!shooting && !shooting2 && !centerControl && indexerState != -1) {
        switch (indexerState) {
            case 0: indexer.setPosition(pos1Intake); break;
            case 1: indexer.setPosition(pos2Intake); break;
            case 2: indexer.setPosition(pos3Intake); break;
        }

        // COLOR DETECTION: Check what color ball was just picked up
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        if (count(pattern, x1) > 0 && ...) {
            if (result.closestSwatch == ... && colorTime.milliseconds() > 500) {
                pattern = pattern.substring(0, indexerState) + "G"
                        + pattern.substring(indexerState + 1);
                colorTime.reset();
            } else if (...) {
                // same for purple
            }
        }
    }

    // Telemetry
    telemetry.addData("path state", pathState);
    telemetry.addData("x", follower.getPose().getX());
    // ... more telemetry
    telemetry.update();
}
```

This single method contains: hood control, follower updates, turret tracking,
shooter control, shooting logic, intake control, center control pre-positioning,
indexer positioning, color detection, AND telemetry. **~100 lines** of
interleaved logic.

### How it was modified

```java
// In BaseAutonomous.java:
@Override
public void loop() {
    hw.hoodExtension.setPosition(0);           // 1. Hood
    follower.update();                          // 2. Path following
    autonomousPathUpdate();                     // 3. Path state machine
    turret.update(                              // 4. Turret tracking
        follower.getPose().getX(),
        follower.getPose().getY(),
        follower.getHeading(),
        telemetry, gamepad1, gamepad2
    );
    hw.shooter.setVelocity(RobotHardware.SHOOTER_VELOCITY);  // 5. Shooter
    if (shooter.isShooting()) shooter.update(telemetry);      // 6. Shooting SM
    intake.run(true);                                          // 7. Intake
    indexerSub.update(shooter.isShooting(), false,             // 8. Indexer
        turret.getMotif(), telemetry);

    // 9. Sync pattern after shooting
    if (!shooter.isShooting() && "XXX".equals(shooter.getPattern())) {
        indexerSub.setPattern("XXX");
        indexerSub.setCenterControl(false);
    }

    // 10. SharedState for teleop handoff
    SharedState.xPos = follower.getPose().getX();
    SharedState.yPos = follower.getPose().getY();
    // ...

    // 11. Telemetry
    telemetry.addData("Path State", pathState);
    // ...
    telemetry.update();
}
```

### Why

| Old (JWBF.loop()) | New (BaseAutonomous.loop()) | Benefit |
|---|---|---|
| 100+ lines of mixed logic | ~30 lines of method calls | Easy to read the orchestration flow at a glance |
| Center control logic (15 lines) inline | Inside `IndexerSubsystem.update()` | Logic is near the data it operates on |
| Color detection logic (20 lines) inline | Inside `IndexerSubsystem.update()` | Reusable in teleop too |
| `turretTracker(true)` accessing fields directly | `turret.update(x, y, heading, ...)` with explicit params | Clear data flow, no hidden dependencies |
| `automated_shoot(shooting)` -- 340 lines triggered by boolean | `shooter.update(telemetry)` -- state machine manages itself | Self-contained, predictable behavior |
| SharedState commented out (never saved data!) | SharedState explicitly updated every loop | Data actually transfers to teleop |
| All in ONE method | Each line delegates to a subsystem | If intake breaks, look at IntakeSubsystem, not the 100-line loop |

**Key concept -- Orchestration vs Implementation**:
The `loop()` method should be the *conductor* of the orchestra -- it tells each
instrument (subsystem) when to play, but it doesn't contain the music (logic)
itself. In the old code, the conductor was also playing violin, drums, and
trumpet simultaneously.

---

## 13. The init() Method (Lines 940-1046)

### What the old code does

About 100 lines of hardware initialization:

```java
@Override
public void init() {
    pathTimer = new Timer();
    opmodeTimer = new Timer();
    opmodeTimer.resetTimer();

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(startPose);
    buildPaths();

    // Limelight setup (5 lines)
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    limelight.pipelineSwitch(0);
    limelight.start();

    // Color sensor setup (12 lines)
    colorSensor = new PredominantColorProcessor.Builder()
        .setRoi(ImageRegion.asUnityCenterCoordinates(0.2, -0.5, 0.4, -0.8))
        .setSwatches(...)
        .build();
    VisionPortal portal = new VisionPortal.Builder()...build();

    // Drive motors (8 lines)
    frontLeftMotor = hardwareMap.get(DcMotor.class, "flm");
    frontRightMotor = hardwareMap.get(DcMotor.class, "frm");
    // ...directions...

    // Turret setup (7 lines)
    turret = hardwareMap.get(DcMotor.class, "turret");
    turret.setDirection(DcMotorSimple.Direction.REVERSE);
    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    turret.setTargetPosition(0);
    turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    turret.setPower(1);

    // Shooter setup (4 lines)
    shooter1 = hardwareMap.get(DcMotorEx.class, "shoot1");
    shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
    shooter1.setPIDFCoefficients(...);

    // Servos (3 lines)
    hoodExtension = hardwareMap.get(Servo.class, "s1");
    indexer = hardwareMap.get(Servo.class, "index");
    hinge = hardwareMap.get(Servo.class, "h");

    // Intake (2 lines)
    intake = hardwareMap.get(DcMotor.class, "intake");
    intake.setDirection(DcMotorSimple.Direction.REVERSE);

    // IMU (5 lines)
    IMU imu = hardwareMap.get(IMU.class, "imu");
    // ...

    // 10+ unused local variables declared but never used!
    boolean intakeToggle = false;
    int loop = 0;
    String manual_shoot = "";
    // ...

    // Initial servo positions
    hinge.setPosition(0.09);
    hoodExtension.setPosition(0);
}
```

**This entire block was copy-pasted into every OpMode file** (with slight
variations for JWRC's different motor names or indexer positions).

### How it was modified

All hardware initialization moved to `RobotHardware.java`:

```java
// RobotHardware.java:
public void init() {
    // All 60+ lines of hardwareMap.get() calls are HERE, ONCE
    frontLeftMotor = hardwareMap.get(DcMotor.class, "flm");
    // ... everything else ...
    hinge.setPosition(HINGE_CLOSED);
    hoodExtension.setPosition(0);
}
```

The autonomous `init()` in `BaseAutonomous.java` becomes:

```java
@Override
public void init() {
    pathTimer = new Timer();
    opmodeTimer = new Timer();
    opmodeTimer.resetTimer();

    hw = new RobotHardware(hardwareMap);
    hw.init();                              // <-- ONE LINE replaces 60+

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(getStartPose());

    // Create subsystems (passing hardware references)
    AllianceConfig alliance = getAlliance();
    IndexerConfig idxConfig = getIndexerConfig();
    double[] delays = getShootingDelays();

    intake = new IntakeSubsystem(hw.intake);
    turret = new TurretSubsystem(hw.turret, hw.limelight,
        alliance.targetX, alliance.targetY);
    shooter = new ShootingStateMachine(hw.indexer, hw.hinge, idxConfig,
        delays[0], delays[1], delays[2]);
    indexerSub = new IndexerSubsystem(hw.indexer, hw.colorSensor,
        idxConfig, getInitialPattern());

    buildPaths(follower);
}
```

### Why

| Problem | Solution |
|---|---|
| 60+ lines of hardware init duplicated across 9 files | `RobotHardware.init()` -- write it ONCE |
| Changing a hardware name (e.g., "flm" to "frontLeft") requires editing 9 files | Change it in ONE file |
| 10+ unused variables declared in init() | Removed entirely (dead code elimination) |
| No documentation about what hardware names mean | `RobotHardware` has extensive Javadoc comments |
| Motor directions set inconsistently across files | ONE source of truth for all motor directions |
| PIDF coefficients scattered across files | `RobotHardware` documents what P=160, I=0, D=0, F=15 means |

**Key concept -- Dependency Injection**:
Notice how `BaseAutonomous.init()` creates each subsystem by *passing in* the
hardware references it needs:
```java
turret = new TurretSubsystem(hw.turret, hw.limelight, alliance.targetX, alliance.targetY);
```
The turret subsystem doesn't reach into the hardware map itself -- it receives
what it needs through its constructor. This makes it:
1. **Testable**: You could pass in mock/fake hardware for testing.
2. **Explicit**: By reading the constructor, you know exactly what hardware the
   turret uses.
3. **Flexible**: The same subsystem class works with any motor/camera, not just
   specific hardware names.

---

## 14. Lifecycle Methods: start() and stop() (Lines 1057-1067)

### What the old code does

```java
@Override
public void start() {
    follower.activateAllPIDFs();
    setPathState(0);
}

@Override
public void stop() {}
```

`start()` activates the PedroPathing PID controllers and sets the initial path
state. `stop()` is empty because the FTC SDK automatically stops all motors and
servos when an OpMode ends.

### How it was modified

Moved unchanged to `BaseAutonomous.java`:

```java
@Override
public void start() {
    follower.activateAllPIDFs();
    setPathState(0);
}

@Override
public void stop() {
    // Everything auto-disables via FTC SDK
}
```

### Why

These are identical across all autonomous OpModes, so they belong in the base
class. A subclass can override them if it needs custom start/stop behavior
(none currently do).

---

## 15. Summary of All Changes

### Lines of code comparison

| Metric | Old (JWBF.java alone) | New (BlueFarAuto.java) | Where the rest went |
|---|---|---|---|
| Total lines | 1,073 | ~280 | Base class + subsystems |
| Hardware init | ~60 lines | 0 (inherited) | `RobotHardware.java` (~200 lines, shared by ALL OpModes) |
| Shooting logic | ~340 lines | 0 (inherited) | `ShootingStateMachine.java` (~140 lines of logic) |
| Turret tracking | ~70 lines | 0 (inherited) | `TurretSubsystem.java` (~120 lines) |
| Color detection + indexer | ~40 lines | 0 (inherited) | `IndexerSubsystem.java` (~100 lines) |
| count() utility | ~8 lines | 0 (shared) | `PatternUtil.java` (~30 lines) |
| Intake control | ~8 lines | 0 (inherited) | `IntakeSubsystem.java` (~25 lines) |
| Poses + paths | ~110 lines | ~110 lines | Stays in BlueFarAuto (unique per auto) |
| State machine | ~150 lines | ~120 lines | Stays in BlueFarAuto (unique per auto) |
| Loop orchestration | ~100 lines | 0 (inherited) | `BaseAutonomous.java` (~60 lines) |
| Variables/timers | ~80 lines | ~20 lines | Distributed to subsystems |

### Across ALL 9 original files

| Before | After |
|---|---|
| ~9,000 total lines across 9 files | ~3,500 total lines across 29 files |
| ~5,500 duplicated lines eliminated | 0 duplicated lines |
| count() written 9 times | Written 1 time |
| Hardware init written 9 times | Written 1 time |
| automated_shoot() written 9 times | Written 1 time |
| turretTracker() written 9 times | Written 1 time |

### Architecture changes

| Principle | Old | New |
|---|---|---|
| **DRY** (Don't Repeat Yourself) | Massive duplication | Zero duplication |
| **SRP** (Single Responsibility) | Each file does everything | Each class does one thing |
| **Open/Closed** | Adding a new auto = copy-paste 1000 lines | Extend BaseAutonomous, implement 7 methods |
| **Encapsulation** | All variables accessible everywhere | Each subsystem hides its internal state |
| **Named Constants** | Magic numbers like `0.4`, `0.09`, `742`, `1120` | `HINGE_OPEN`, `HINGE_CLOSED`, `TURRET_TICKS_PER_DEGREE`, `SHOOTER_VELOCITY` |

---

## 16. Discussion Questions for the Team

Work through these questions together as a team. There are no wrong answers --
the goal is to deepen your understanding of the code and software design!

### Understanding the Old Code

1. **Counting exercise**: How many times was the exact same `count()` method
   written in the old codebase? What if there was a bug in one copy but not
   the others?

2. **Spot the difference**: Compare the `turretTracker()` in JWBF.java vs
   JWRC.java. The angle formula is different -- can you figure out WHY they
   used different math? (Hint: which corner of the field is the basket on?)

3. **Timer confusion**: In `automated_shoot()`, there are three timers:
   `indexerTime`, `hingeTime`, and `hinge12`. Can you trace through the code
   to figure out what each one measures? Why are they all reset at the same
   time, even though they measure different things?

### Understanding the New Code

4. **State machine tracing**: Start with `ShootingStateMachine` in state
   `IDLE`. Call `startShooting("GPP", "PGP", false)`. Trace through every
   state transition until the machine returns to IDLE. Write down each state
   and what happens in it.

5. **Motif alignment**: If motif = "PPG" and pattern = "GPP":
   - What is `motifGreenPos`?
   - What is `patternGreenPos`?
   - Which case in `calculateShootOrder()` is triggered?
   - What servo positions are returned?

6. **Base class benefits**: Imagine we need to add a new autonomous routine
   for a different starting position. With the old code, what files would you
   need to copy and modify? With the new code, what do you need to do?

### Design Thinking

7. **What would you add?** Now that the code is modular, what new features
   would you add? Some ideas:
   - Automatic detection of which indexer config to use
   - A "panic button" that immediately stops shooting and resets
   - Telemetry logging to a file for post-match analysis
   - An auto-selector on the Driver Station that doesn't require separate
     OpModes for each starting position

8. **Testing**: Look at the test suite in the `test/` package. Pick one test
   (e.g., `ShooterTest`) and think about what it's testing. What are the
   most important things to verify about the shooter? What test would YOU
   add?

9. **Error handling**: What happens in the current code if the Limelight
   doesn't detect any AprilTags? Trace through `TurretSubsystem.update()`
   -- where does it aim? Is this a good fallback strategy?

10. **Naming matters**: Look at these variable names from the old code:
    `flag`, `flag1`, `flag2`, `hinge12`, `shooting2`, `x1`. Can you think
    of better names that explain what they represent? (The new code has
    already renamed most of these -- compare!)

---

*This guide was created as part of the TaigaBots 18190 code modularization
project. Good luck in the 2025-2026 season!*
