# Pit Checklist

Run this before every match, in order. It takes about two minutes. Every item names what to look
at, so it can be done by someone who did not write the code.

The first item is the one M14 was built around: **the robot tells you what is wrong with it**, by
name, on the dashboard. Look before you look for anything else.

## 1. The Alerts widget (Driver Station dashboard, "Alerts")

- [ ] **No red errors.** Every disconnected device is named with its CAN ID and bus, for example
      `Flywheel follower motor (CAN 26, rio) disconnected`. Find that plug.
- [ ] **`Faults/Blocking` is empty** (AdvantageScope, or the dashboard). Anything listed there stops
      the robot shooting: a flywheel motor, the turret motor, or an unknown roboRIO. The robot will
      still drive and intake. Fix it or accept that it will not score.
- [ ] **No "Unknown roboRIO".** That means the serial is not entered as `kCompRoboRioSerial` or
      `kPracticeRoboRioSerial`, the robot is running the competition robot's encoder offsets, and it
      will not shoot until the serial is entered and the code redeployed.
- [ ] **No "Tuning mode is ON".** Tuning mode must be off at an event. Redeploy the competition
      build (`feature-comp_ready` or its successor).
- [ ] **No "PathPlanner path unreadable".** The selected auto would skip that leg.
- [ ] Warnings (yellow) are allowed, but read them: a turret absolute encoder that is down means a
      slow homing, and a QuestNav warning means the headset is not being trusted.

## 2. Power

- [ ] Fresh battery. At rest, `Power/FilteredVoltage` reads at least 12.5 V.
- [ ] `Power/Severity` reads 0 at rest.
- [ ] Main breaker and roboRIO power leads seated.

## 3. The start of autonomous

- [ ] The intake is **stowed** and **8 pieces** are loaded, as the rules require.
- [ ] The routine is selected in "Auto Choice", and `AutoSelector/Selected` names it.
- [ ] Place the robot until `AutoSelector/StartPoseError` reads **under 5 cm**. The warning "Robot is
      not on the selected auto's start" fires above 15 cm.
- [ ] On the field, check the Driver Station shows the right alliance: every path and the goal flip
      with it.

## 4. Mechanisms, by hand, robot disabled

- [ ] The turret moves freely by hand through its arc, and its cable is not snagged.
- [ ] The intake pivot holds itself up: it is in brake, and it must not drop onto the turret.
- [ ] The flywheel spins freely and both wheels turn together.
- [ ] Bumpers on, the right colour, and tight.

## 5. After the match

- [ ] Wait about five seconds after the robot is disabled: the drivetrain drops to coast
      (`Drive/BrakeMode` false) so the robot can be pushed. The intake and turret stay in brake.
- [ ] Pull the log from the USB stick if anything went wrong. `Faults/*`, `Power/*`, `LoopTimeMs`
      and `Auto/*` are the first keys to look at.
