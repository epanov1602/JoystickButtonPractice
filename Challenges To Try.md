# Challenges

## Challenge 1
Go to the end of `initialize()` function in `RobotContainer` class, and make a command for a robot to turn right three times. Attach it to button 4.
```
    Command rotateRightTwoTimes = new TurnDegrees(0.4, -360, m_drivetrain);
    JoystickButton button4 = new JoystickButton(m_joystick, 4);
    button4.onTrue(rotateRightTwoTimes);
```

## Challenge 2
In addition to hooking that command button 4, also assign it to be the autonomous-mode command. And test that it works.
```
    m_autonomousCommand = rotateRightTwoTimes;
```

## Challenge 3 (putting multiple commands together)
Make the command to go to point {x=40, y=10} and then turn around to face East (90 degrees heading). And attach this command to button 2.
```
   Command goNorthAndTurnSouth = new SequentialCommandGroup(
      new GoToPoint(m_drivetrain, 40, 10, 2),
      new AimToDirection(m_drivetrain, 90)
    );
    JoystickButton button2 = new JoystickButton(m_joystick, 2);
    button2.onTrue(goNorthAndTurnSouth);
```

## Challenge 4
Make the command for a robot to run triangular race on a 40x40 square, going through three vertices: (40, 0), (40, 40), (0, 0). And attach this command to button 3.
```
// guess yourself!
```

## Challenge 6
Change the command above to run the same race two times, and aim north (heading=0) at the end of the race.

Also, in addition to hooking that command to button 3, make this the autonomous command as you did in challenge 2.

## Race Challenge
Now try to change (hack!) the code of GoToPoint command, for the robot to complete that triangular race as fast as you can.