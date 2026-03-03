# TaigaBots 18190 -- Git Branching & Pull Request Guide
## A Student-Friendly Guide to Safe Collaboration

This guide teaches the team how to use Git branches and Pull Requests (PRs)
to work on code safely without breaking the `main` branch.

---

## Table of Contents

1. [The Golden Rule](#1-the-golden-rule)
2. [Key Concepts (5-Minute Crash Course)](#2-key-concepts-5-minute-crash-course)
3. [Our Branching Strategy](#3-our-branching-strategy)
4. [Step-by-Step: Creating Your First Branch](#4-step-by-step-creating-your-first-branch)
5. [Step-by-Step: Making Commits](#5-step-by-step-making-commits)
6. [Step-by-Step: Pushing and Creating a Pull Request](#6-step-by-step-pushing-and-creating-a-pull-request)
7. [Step-by-Step: Reviewing a Pull Request](#7-step-by-step-reviewing-a-pull-request)
8. [Step-by-Step: Merging a Pull Request](#8-step-by-step-merging-a-pull-request)
9. [Common Scenarios with Examples](#9-common-scenarios-with-examples)
10. [Handling Merge Conflicts](#10-handling-merge-conflicts)
11. [Quick Reference Cheat Sheet](#11-quick-reference-cheat-sheet)
12. [Rules for Our Team](#12-rules-for-our-team)

---

## 1. The Golden Rule

> **NEVER commit directly to `main`.**
>
> Always create a branch, make your changes there, and use a Pull Request
> to merge into `main` after someone else reviews it.

Why? Because `main` is what gets deployed to the robot. If broken code gets
into `main`, the robot stops working at competition. Branches let you
experiment safely.

---

## 2. Key Concepts (5-Minute Crash Course)

### What is a Branch?

Think of `main` as the "published book." A **branch** is your personal
"rough draft" where you can scribble, erase, and rewrite without affecting
the published book. When your draft is ready, you submit it for review
(a Pull Request), and if approved, it gets added to the published book.

```
main:          A --- B --- C --- D          (stable, tested code)
                      \         /
your-branch:           E --- F             (your work-in-progress)
```

### What is a Commit?

A **commit** is a snapshot of your changes with a message describing what
you did. Think of it as "saving your game" -- you can always go back to
any previous save.

### What is a Pull Request (PR)?

A **Pull Request** is a formal request to merge your branch into `main`.
It shows everyone what you changed, lets teammates review and comment,
and creates a record of why changes were made. On GitHub, it looks like
a conversation thread with your code changes attached.

### What is a Merge?

**Merging** takes the changes from your branch and combines them into
`main`. After merging, your changes become part of the stable codebase.

---

## 3. Our Branching Strategy

We use a simple strategy called **"Feature Branching"**:

```
main (always stable, always works on the robot)
 |
 +-- feature/modular-refactor      (big changes like our refactor)
 |
 +-- feature/new-auto-routine      (adding a new autonomous)
 |
 +-- fix/turret-angle-bug          (fixing a specific bug)
 |
 +-- test/shooter-velocity-tuning  (testing/tuning changes)
```

### Branch Naming Convention

Use this format: `type/short-description`

| Type | When to Use | Example |
|------|-------------|---------|
| `feature/` | Adding new functionality | `feature/add-parking-auto` |
| `fix/` | Fixing a bug | `fix/indexer-color-misdetect` |
| `test/` | Tuning or testing changes | `test/shooter-velocity-2000` |
| `docs/` | Documentation only | `docs/add-code-comments` |
| `refactor/` | Restructuring code without changing behavior | `refactor/modular-architecture` |

### Rules

- `main` = always works, always tested on the robot
- Feature branches = work in progress, may be broken
- Every branch gets a Pull Request before merging
- At least 1 teammate must review before merging
- Delete branches after they're merged (keeps things clean)

---

## 4. Step-by-Step: Creating Your First Branch

### In Android Studio (Recommended for Beginners)

1. **Open the Git menu**: Bottom-right corner of Android Studio, click
   the branch name (probably says "main")
2. Click **"New Branch"**
3. Type your branch name: `refactor/modular-architecture`
4. Make sure **"Checkout branch"** is checked
5. Click **Create**

You're now on your new branch! Any changes you make will ONLY affect
this branch.

### In the Terminal (For Command-Line Users)

```bash
# Step 1: Make sure you're on main and it's up to date
git checkout main
git pull origin main

# Step 2: Create a new branch and switch to it
git checkout -b refactor/modular-architecture

# That's it! You're now on the new branch.
# Verify with:
git branch
# The asterisk (*) shows which branch you're on:
#   main
# * refactor/modular-architecture
```

### How to Switch Between Branches

```bash
# Switch to main
git checkout main

# Switch back to your branch
git checkout refactor/modular-architecture

# See all branches
git branch          # local branches
git branch -a       # all branches (including remote)
```

**WARNING**: Always commit or stash your changes before switching branches!
Uncommitted changes can follow you to the wrong branch.

---

## 5. Step-by-Step: Making Commits

### Good Commit Messages

Write commit messages that explain **WHY**, not just what:

```
GOOD:
  "Extract turret tracking into TurretSubsystem to eliminate duplication"
  "Fix indexer detecting same ball twice by adding 500ms debounce"
  "Add ShooterTest for tuning velocity at competition"

BAD:
  "updated code"
  "fixed stuff"
  "asdfgh"
  "WIP"
```

### Commit Message Format

```
<type>: <short summary in present tense>

<optional longer explanation>
```

Examples:
```
feat: add enum-based shooting state machine

Replaces the 340-line nested if/else in automated_shoot() with a clean
state machine using 7 named states. Reduces code from 12 repeated blocks
to a single parameterized loop.
```

```
fix: correct turret angle formula for red alliance

The old code used a different atan formula in JWRC vs JWBF. Unified to
((rawAngle % 180) + 180) % 180 which works for both field corners.
```

### In Android Studio

1. **Open the Commit window**: `Ctrl+K` (or Git menu > Commit)
2. Check the boxes next to files you want to include
3. Write your commit message in the text box
4. Click **Commit** (NOT "Commit and Push" -- we'll push later)

### In the Terminal

```bash
# See what files changed
git status

# Add specific files (PREFERRED - you control exactly what goes in)
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/TurretSubsystem.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShootingStateMachine.java

# Or add all files in a specific folder
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/

# Create the commit
git commit -m "feat: extract turret and shooting into subsystem classes"

# Verify
git log --oneline -5
```

### How Often to Commit?

Commit **early and often**. A good rule: commit whenever you've completed
one logical piece of work that compiles and doesn't break things.

Good commit sizes:
- "Added RobotHardware class" (one complete new class)
- "Moved count() to PatternUtil" (one specific refactor)
- "Fixed hinge timing in ShootingStateMachine" (one bug fix)

Bad commit sizes:
- "Did everything" (way too big -- impossible to review)
- "Changed one semicolon" (too small unless it's a real bug fix)

---

## 6. Step-by-Step: Pushing and Creating a Pull Request

### Push Your Branch to GitHub

```bash
# First time pushing a new branch:
git push -u origin refactor/modular-architecture

# After that, just:
git push
```

The `-u` flag sets up "tracking" so future `git push` commands know
where to go.

### Create a Pull Request on GitHub

1. Go to https://github.com/TaigaBots18190/2025-2026-DECODE
2. You'll see a yellow banner: **"refactor/modular-architecture had recent
   pushes -- Compare & pull request"**. Click it!
3. Fill in the PR:

**Title**: `Modularize codebase into subsystems, base classes, and test suite`

**Description** (use this template):

```markdown
## What Changed
- Extracted hardware init into RobotHardware class
- Created subsystem classes (TurretSubsystem, ShootingStateMachine, etc.)
- Created BaseAutonomous abstract class
- Added test suite with scrollable menu
- Added @Disabled to old files (they still exist but won't show on Driver Station)

## Why
- Eliminated ~5,500 lines of duplicated code
- Made it possible to add new autonomous routines in ~100 lines instead of 1,000
- Each component can now be tested individually

## How to Test
1. Build the project in Android Studio (should compile with no errors)
2. Deploy to robot
3. Verify old OpModes show as disabled on Driver Station
4. Run "Blue Far" autonomous -- should behave identically to old "JWBF"
5. Run "Test Menu" teleop -- verify all individual tests work
6. Run "TeleOp Blue" -- verify teleop works

## Files Changed
- 29 new files created (see TeamCode/CODE_COMPARISON_GUIDE.md for details)
- 13 old files have @Disabled added (NOT deleted)
```

4. On the right sidebar, set:
   - **Reviewers**: Pick 1-2 teammates
   - **Labels**: Add "enhancement" or "refactor"
5. Click **"Create Pull Request"**

### Using the GitHub CLI (Alternative)

```bash
gh pr create \
  --title "Modularize codebase into subsystems and base classes" \
  --body "## What Changed
- Extracted hardware init into RobotHardware
- Created subsystem classes
- Created BaseAutonomous abstract class
- Added test suite with scrollable menu

## How to Test
1. Build in Android Studio
2. Deploy to robot
3. Run Blue Far auto and compare to old JWBF"
```

---

## 7. Step-by-Step: Reviewing a Pull Request

When a teammate creates a PR, here's how to review it:

### Step 1: Check Out the Branch Locally

```bash
# Fetch the latest branches from GitHub
git fetch origin

# Check out the PR branch
git checkout refactor/modular-architecture

# Build and test on the robot!
```

### Step 2: Review on GitHub

1. Go to the PR page on GitHub
2. Click the **"Files changed"** tab
3. Read through the changes:
   - Green lines = new code added
   - Red lines = old code removed
4. **Leave comments** on specific lines by clicking the `+` icon
5. Ask questions! Examples:
   - "Why did we use 500ms here instead of 1000ms?"
   - "Should this constant be in RobotHardware instead?"
   - "What happens if the Limelight isn't connected?"

### Step 3: Test on the Robot

This is the most important step! Before approving:

- [ ] Does the code compile without errors?
- [ ] Does "Blue Far" autonomous work the same as old "JWBF"?
- [ ] Does "Red Center" autonomous work the same as old "JWRC"?
- [ ] Does TeleOp work correctly?
- [ ] Does the Test Menu show all tests?
- [ ] Do individual tests (shooter, turret, etc.) run correctly?

### Step 4: Approve or Request Changes

On GitHub:
1. Click **"Review changes"** (green button, top right of Files Changed)
2. Choose one:
   - **Approve**: "Looks good, tested on robot, everything works!"
   - **Request changes**: "Found a bug -- the turret aims wrong on Red side"
   - **Comment**: "I have questions but haven't tested yet"
3. Click **"Submit review"**

---

## 8. Step-by-Step: Merging a Pull Request

Once the PR has at least 1 approval:

### On GitHub

1. Go to the PR page
2. Click **"Merge pull request"** (green button at the bottom)
3. Choose **"Create a merge commit"** (keeps the history clean)
4. Click **"Confirm merge"**
5. Click **"Delete branch"** (cleans up the remote branch)

### After Merging -- Everyone Updates Their Local Code

Every team member should run:

```bash
git checkout main
git pull origin main
```

This downloads the newly merged changes to their computer.

---

## 9. Common Scenarios with Examples

### Scenario 1: Checking In the Modular Refactor (Right Now!)

This is what we need to do with the current changes:

```bash
# 1. Make sure you're on main
git checkout main

# 2. Create the branch
git checkout -b refactor/modular-architecture

# 3. Add the new files (organized by category)

# Utility classes
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/

# Hardware classes
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/

# Subsystem classes
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/

# Autonomous classes
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/

# TeleOp classes
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/

# Test classes
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/

# Documentation
git add TeamCode/CODE_COMPARISON_GUIDE.md
git add TeamCode/GIT_BRANCHING_GUIDE.md

# Modified old files (with @Disabled added)
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/JWBF.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/JWBF_Finish.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/JWRC.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/JWBC.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/JWRF.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/jwb.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/jwr.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/BobRedF.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/BobRedC.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tom_Red_F.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tom_Red_C.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tom_Blue_F.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/ServoTester1.java

# Deprecated copies
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/deprecated/

# 4. Verify what's staged
git status

# 5. Commit
git commit -m "refactor: modularize codebase into subsystems, base classes, and test suite

- Extract hardware init into RobotHardware (eliminates 60 lines x 9 files)
- Create ShootingStateMachine with enum states (replaces 340-line if/else)
- Create TurretSubsystem with unified angle formula for both alliances
- Create BaseAutonomous abstract class (shared loop/init orchestration)
- Create IndexerSubsystem (color detection + positioning)
- Create IntakeSubsystem, PatternUtil, AllianceConfig, IndexerConfig
- Add 9 individual hardware tests with scrollable TestMenuOpMode
- Add @Disabled to all 13 old files (preserved, not deleted)
- Add CODE_COMPARISON_GUIDE.md for student reference"

# 6. Push to GitHub
git push -u origin refactor/modular-architecture

# 7. Create the Pull Request (using GitHub CLI if installed, or do it on GitHub)
# gh pr create --title "Modularize codebase" --body "See description..."
```

### Scenario 2: Student Adds a New Autonomous Routine

```bash
# 1. Start from the latest main
git checkout main
git pull origin main

# 2. Create a feature branch
git checkout -b feature/add-red-far-finish-auto

# 3. Create the new file:
#    autonomous/RedFarFinishAuto.java
#    (extend BaseAutonomous, implement the 7 abstract methods)

# 4. Commit
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/RedFarFinishAuto.java
git commit -m "feat: add Red Far Finish autonomous routine

Extends BaseAutonomous with RED alliance, CONFIG_B indexer,
and 1000/1175/1350ms shooting delays. Includes gate-pushing path."

# 5. Push and create PR
git push -u origin feature/add-red-far-finish-auto
# Then create PR on GitHub
```

### Scenario 3: Student Fixes a Bug Found During Testing

```bash
# 1. Start from main
git checkout main
git pull origin main

# 2. Create a fix branch
git checkout -b fix/turret-overshooting-on-red

# 3. Edit the file, fix the bug
# (edit TurretSubsystem.java)

# 4. Commit with a clear explanation
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/TurretSubsystem.java
git commit -m "fix: turret overshooting target on red alliance side

The angle calculation was not accounting for the robot facing away
from the basket. Added heading normalization to prevent the turret
from spinning past its physical limits."

# 5. Push and create PR
git push -u origin fix/turret-overshooting-on-red
```

### Scenario 4: Student Tunes Servo Positions at Competition

```bash
# Quick tuning branch
git checkout main
git pull origin main
git checkout -b test/tune-indexer-config-b

# Edit IndexerConfig.java with new values
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/IndexerConfig.java
git commit -m "tune: adjust CONFIG_B indexer positions after field testing

pos2Intake changed from 0.4695 to 0.472 (was slightly misaligned)
pos3Shoot changed from 0.5706 to 0.568 (balls were hitting rim)"

git push -u origin test/tune-indexer-config-b
```

---

## 10. Handling Merge Conflicts

Merge conflicts happen when two people edit the same lines in the same
file. Don't panic! Here's how to handle them:

### When Do Conflicts Happen?

```
main:         ... edit line 50 of TurretSubsystem.java ...
your-branch:  ... also edit line 50 of TurretSubsystem.java ...

When you try to merge: CONFLICT! Git doesn't know which change to keep.
```

### How to Resolve

```bash
# 1. Update your branch with the latest main
git checkout your-branch-name
git fetch origin
git merge origin/main

# 2. Git will say "CONFLICT in TurretSubsystem.java"
# 3. Open the file -- you'll see conflict markers:

<<<<<<< HEAD
    double deadband = 2.0;  // Your version
=======
    double deadband = 1.5;  // Main's version
>>>>>>> origin/main

# 4. Choose the correct version (or combine both):
    double deadband = 1.5;  // Use the tuned value from main

# 5. Remove the conflict markers (<<<, ===, >>>)

# 6. Stage and commit the resolution
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/TurretSubsystem.java
git commit -m "resolve: merge conflict in TurretSubsystem deadband value"

# 7. Push
git push
```

### In Android Studio

Android Studio has a visual merge tool:
1. When a conflict occurs, you'll see a notification
2. Click **"Resolve"**
3. A 3-panel view opens: Left (yours) | Center (result) | Right (theirs)
4. Click the arrows to accept changes from either side
5. Click **"Apply"**

### How to AVOID Conflicts

- **Pull main often**: Before starting work each day, update your branch
- **Keep branches small**: Smaller changes = fewer conflicts
- **Communicate**: If two people are editing the same file, coordinate!
- **Don't both edit the same file at the same time** if possible

---

## 11. Quick Reference Cheat Sheet

### Daily Workflow

```bash
# Start of day: get latest code
git checkout main
git pull origin main

# Create branch for today's work
git checkout -b feature/my-task-today

# ... do your work ...

# Commit your changes
git add <files>
git commit -m "feat: description of what you did"

# Push to GitHub
git push -u origin feature/my-task-today

# Create a PR on GitHub website
```

### Essential Commands

| Command | What It Does |
|---------|-------------|
| `git status` | Show what files changed |
| `git branch` | List branches (asterisk = current) |
| `git checkout main` | Switch to main branch |
| `git checkout -b name` | Create new branch and switch to it |
| `git pull origin main` | Download latest main from GitHub |
| `git add file.java` | Stage a file for commit |
| `git add folder/` | Stage entire folder |
| `git commit -m "msg"` | Save staged changes with message |
| `git push` | Upload commits to GitHub |
| `git push -u origin branch-name` | First push of a new branch |
| `git log --oneline -10` | Show last 10 commits |
| `git diff` | Show unstaged changes |
| `git stash` | Temporarily save uncommitted changes |
| `git stash pop` | Restore stashed changes |

### Emergency Commands

```bash
# "I committed to main by accident!"
# (Don't panic. Create a branch from where you are)
git checkout -b fix/accidental-main-commit
git push -u origin fix/accidental-main-commit
# Then ask a mentor to help reset main

# "I want to undo my last commit (keep the changes)"
git reset --soft HEAD~1

# "I want to see what a file looked like before I changed it"
git show HEAD:path/to/file.java

# "I want to throw away all my uncommitted changes" (CAREFUL!)
git checkout -- path/to/file.java
```

---

## 12. Rules for Our Team

### The 5 Rules

1. **Never push directly to `main`** -- Always use branches and PRs
2. **Every PR needs at least 1 review** -- A second pair of eyes catches bugs
3. **Test on the robot before approving** -- Code that compiles isn't necessarily code that works
4. **Write meaningful commit messages** -- Future you will thank present you
5. **Delete branches after merging** -- Keep the branch list clean

### PR Review Checklist

Before approving any Pull Request, verify:

- [ ] Code compiles without errors in Android Studio
- [ ] No hardcoded magic numbers (use named constants)
- [ ] New files have documentation comments
- [ ] Old files weren't accidentally deleted (only @Disabled)
- [ ] Tested on the actual robot (autonomous AND teleop)
- [ ] Commit messages are descriptive

### Branch Protection (Ask a Mentor to Set Up)

On GitHub, go to Settings > Branches > Add rule:
- Branch name pattern: `main`
- Check: "Require a pull request before merging"
- Check: "Require approvals" (set to 1)
- Check: "Dismiss stale pull request approvals when new commits are pushed"

This makes it **impossible** to push directly to main, even by accident.

---

## Visual Summary

```
                        GitHub (Remote)
                    =====================

    main ----A----B----C-----------M------>  (always stable)
                        \         /
    feature/xyz          D---E---F           (your work)
                              |
                         Pull Request
                        "Review please!"
                              |
                     Teammate approves
                              |
                         Merge (M)


                     Your Computer (Local)
                    ========================

    1. git checkout main          (start from stable code)
    2. git pull origin main       (get latest from GitHub)
    3. git checkout -b feature/x  (create your workspace)
    4. ... write code ...
    5. git add + git commit       (save your work)
    6. git push -u origin feat/x  (upload to GitHub)
    7. Create PR on GitHub        (request review)
    8. Teammate reviews + approves
    9. Merge on GitHub
   10. Everyone: git pull origin main  (get the merged code)
```

---

*Remember: branches are free, merging is easy, but debugging broken main
at 8am on competition day is NOT fun. Use branches!*
