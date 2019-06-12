[statusPic]: status.png "cywwtaip"

![statusPic][statusPic]

# cywwtaip
Study Project 2 (Real Time Strategy Game): AI Game Client written in Julia

## Task Description
* Three players play in "real time" at the same time.
* The aim of the game is to color as large areas of the game world by "driving over" with their own bots with their own color.
* The globular game world is criss-crossed by trenches that normally limit the movement.
* Each player has control over three bots (numbered 0, 1, 2).
  * Bot 0 (monochrome) has 100% speed
  * Bot 1 (spotted) has a speed of 67% and can move over trenches
  * Bot 2 has a speed of 42% and also colors its immediate neighborhood
* The speed of the bots is fixed and can not be slowed down. Steering is done exclusively by steering (angle in radians with sign).
* Bots do not block each other, but it can be painted over the color of other players.
* The speed of movement of the bots depends linearly (between 20% and 100%) on the energy that expires in 10 seconds. Energy is always filled up immediately then exactly when at least one of its own bot moves in the marked area (x, y or z> 0.94).
* The game ends when the time limit expires. The player with the maximum score has won. If several players have the maximum score, the game ends in a draw.

**Minimum Requirements:**
* Predictive movement (does not have to be A*, but also no ad hoc procedure just about neighborhood)
* Different use of the three bots (not all control the same, ideally - but not mandatory - cooperative, taking advantage of the individual peculiarities)
* Consider your own energy in the strategy

## Documentation
not yet

## Problems
Currently this Project only works with Julia 1.0 because package **JavaCall** is not updated for Julia 1.1 yet.

## Requirements
* Julia 1.0 [download here](https://julialang.org/)
* Julia Packages See [REQUIRE](REQUIRE)
* **cywwtaip.zip** (is from Moodle class **GT1(PÜ) AI for Games and Interactive Systems (PÜ) - SoSe2019** at **HTW** (Hochschule für Technik und Wirtschaft) in **Berlin, Germany**
I do not own this library, so please get permission and file from **Prof. Dr. Lenz, Tobias** first!)

## Installation (Only for Windows)
* Please run batch **install_packages.bat**. Every package should be installed automatically.
* The batch **show_packages.bat** gives you info of currently installed packages

## Tested Systems

### Windows
* Operating System: Windows 10 Home 64-bit (10.0, Build 17134 or newer)
* Processor: Intel(R) Core(TM) i7-4510U CPU @ 2.00GHz (4 CPUs), ~2.0GHz
* Memory: 8192MB RAM
* Graphics Card 1: Intel(R) HD Graphics Family
* Graphics Card 2: NVIDIA GeForce 840M

### Linux
* not tested
