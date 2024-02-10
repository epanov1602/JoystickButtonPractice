# Advanced joystick command mapping

## Problem 1
In class RobotContainer, find initialize() function.
In that function, add a command to reset robot odometry and assign it to Joystick button "Y".
```
    Command resetOdometry = new ResetOdometry(m_drivetrain);
    JoystickButton buttonY = new JoystickButton(m_joystick0, Button.kY.value);
    buttonY.onTrue(resetOdometry);
```

## Problem 2
Create a command for bagel pickup and assign it to button B
```
    Command pickUpBagel = new MockPickupCommand(m_drivetrain);
    JoystickButton buttonB = new JoystickButton(m_joystick0, Button.kB.value);
    buttonB.onTrue(pickUpBagel);
```
, even though this MockPickupCommand is not a real command (until the intake subsystem works fully).

## Problem 3
Create a more useful autonomous command (for shooting a preloaded gamepiece, and for picking up another one).
```
    Command turnAroundToAim = new AimToDirection(m_drivetrain, -135);
    Command shoot = new MockShootCommand(m_drivetrain);
    Command goToBagel2 = new GoToPoint(m_drivetrain, 30, -10, false);
    Command pickUpBagel2 = new MockPickupCommand(m_drivetrain);
    Command goBackToSpeaker = new GoToPoint(m_drivetrain, 10, -20, false);
    Command aimAgain = new AimToDirection(m_drivetrain, -180);
    Command shootAgain = new MockShootCommand(m_drivetrain);
    Command bigAutonomousRoutine = new SequentialCommandGroup(turnAroundToAim, shoot, goToBagel2, pickUpBagel2, goBackToSpeaker, aimAgain, shootAgain);
    m_autonomousCommand = bigAutonomousRoutine;
```

## Problem 4
Create a handy command for bringing the gamepieces back to the location from which we an shoot them into the speaker along the *left* side of the field.
```
    // create a command for following the left side of the field and turning to the speaker to shoot, and assign it to the left bumper
    Command goToLeftSide = new GoToPoint(m_drivetrain, 60, -10, true);
    Command turnSouth1 = new AimToDirection(m_drivetrain, 180);
    Command goToLeftSideCorner = new GoToPoint(m_drivetrain, 10, -10, true);
    Command turnEast = new AimToDirection(m_drivetrain, -90);
    Command goToMiddleLeft = new GoToPoint(m_drivetrain, 10, -40, true);
    Command aimToSpeaker1 = new AimToDirection(m_drivetrain, 180);
    Command shoot1 = new MockShootCommand(m_drivetrain);
    Command leftSideFetchRoutine = new SequentialCommandGroup(goToLeftSide, turnSouth1, goToLeftSideCorner, turnEast, goToMiddleLeft, aimToSpeaker1, shoot1);

    JoystickButton leftBumper = new JoystickButton(m_joystick0, Button.kLeftBumper.value);
    leftBumper.onTrue(leftSideFetchRoutine);
```

## Problem 5
In the command above, make it so that the command only runs until someone pressed "X" button to cancel it.
To accomplish this, modify `leftSideFetchRoutine` before assigning it to a button. Modify this way:
```
    leftSideFetchRoutine = leftSideFetchRoutine.until(() -> m_joystick0.getRawButtonPressed(Button.kX.value) == true);
```

## Challenge problem!
Create a similar command to bring a gamepiece to the speaker, but following the right side of the field instead.
