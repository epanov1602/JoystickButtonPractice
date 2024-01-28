# Command-based robot driving challenges

## Challenge 1
Go to the end of `initialize()` function in `RobotContainer` class (inside `RobotContainer.java` file), and make the following command for a robot to turn right three times. Attach the command to button 4.
```
    Command rotateRightTwoTimes = new TurnDegrees(0.4, 360, m_drivetrain);
    JoystickButton button4 = new JoystickButton(m_joystick, 4);
    button4.onTrue(rotateRightTwoTimes);
```
Please test how well it works: switch your robot to Teleoperated mode, and push that joystick button 4.
(is there a mistake in the code above? will the robot turn twice or just once? can you guess how to fix this?)

## Challenge 2
In addition to hooking that command button 4, also assign it to be the autonomous-mode command. And test that it works.
```
    m_autonomousCommand = rotateRightTwoTimes;
```
Please test how well it works: switch your robot to Autonomous mode.

## Challenge 3 (putting multiple commands together)
Make the command to go to point {x=40, y=10} and then turn around to face East (90 degrees heading). And attach this command to button 2.
```
   Command goNorthAndTurnSouth = new SequentialCommandGroup(new GoToPoint(m_drivetrain, 40, 10, 2),
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

## Challenge 5
Change the command above to run the same race two times (two laps!), and then aim north (heading=0) at the end of the race.

## Challenge 6
In addition to hooking that command to button 3 in challenge 5, also make this two-lap race the autonomous command as you did in challenge 2.

## Challenge 7 (real race, finally)
Now try to change (hack!) the code of GoToPoint command, for the robot to complete that triangular race as fast as you can.

Whose robot will win? 

<br />
<br />
<br />
<br />

-----------

Copyright (c) 2009-2021 FIRST and other WPILib contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of FIRST, WPILib, nor the names of other WPILib
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY FIRST AND OTHER WPILIB CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY NONINFRINGEMENT AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL FIRST OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
