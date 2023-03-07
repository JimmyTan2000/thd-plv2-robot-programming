# Turtlebot3 maze challenges

## Setup

Install the Python library `mako`. The project uses the `mako-render` tool.

```
pip install mako
```

`mako-render` will be placed into `~/.local/bin`. The project requires that `mako-render` command is available in yout `PATH`. To check, try:

```
which mako-render
```

If `mako-render` cannot be found, then exit your terminal, open a new Bash session and try it again.

This behavior stems from the fact that Bash automatically puts `~/.local/bin` into your `PATH`, unless `~/.local/bin` does not exist. `pip` may have created `~/.local/bin` for the first time.

Clone the repository and create worlds and models:

```
cd tb3-maze-challenges
make
```

Then you should see the world files:

```
$ ls
...
world_1_1.sdf
world_2_2.sdf
world_5_5.sdf
...
```

# Challenges

Write programs which solve the following challenges. You can use [Turtlebot3 node template](https://mygit.th-deg.de/gaydos/tb3-ros2-template/-/blob/master/tb3.py) as a starter.

## Challenge 0

Play the challenge using the following command. Note that the first invocation of the simulator downloads some 3D models which may take about 60 seconds.
```sh
./play world_1_1.sdf
```

How much are the minimal distances to the walls? Use the laser distance sensor (LDS).


## Challenge 1

```sh
./play world_1_1.sdf
```

Drive to the red wall as close as you can get and stop without colliding with the wall.

Some questions which could help you along:

- what is the minimal value of the LDS at which you should stop?
- how would you set the speed of the robot?

## Challenge 2

```
./play world_1_1.sdf
```

Drive to the red wall, and stop at a safe distance. Then rotate counter-clockwise and drive close to the wooden wall and stop. Only use the laser distance sensor data. If you cannot do an exact 90 degree rotation, that is fine.

- what is a safe distance where you can rotate without colliding with the red wall?
- assume you are only rotating. How can you calculate your rotation in degrees using laser distance sensor data?
- the laser distance sensor data will probably have some noise. How can you alleviate the noise?


## Challenge 3

```
./play world_1_1.sdf
```

You probably noticed that the laser distance sensor is noisy and driving while measuring can cause additional noise.  The challenge is the same, but instead of using laser distance data use the position and orientation published in `/odom`.

- in Gazebo, every cell on the grid is 1m x 1m
- the topic `/odom` contains both the position and the orientation of the robot
- to convert a quaternion to Euler angles, you can use `from transforms3d.euler import quat2euler`. [API reference for `quat2euler`](https://matthew-brett.github.io/transforms3d/reference/transforms3d.euler.html#quat2euler) could help.


## Challenge 4

```
./play world_2_2.sdf
```

Drive to the cell with the red wall and stop without any collision.

- how can you differentiate between wooden, white, and red walls?
- how do you know in which cell you are?
- how do you know to which cell you can/want to drive?


## Challenge 5

```
./play world_5_5.sdf
```

Stop at the cell with the red wall.

optional: how much calories per portion does the chocolate have?
