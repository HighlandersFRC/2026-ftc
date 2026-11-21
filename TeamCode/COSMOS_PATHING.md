# Cosmos paths in FTC

Open the Cosmos project saved at `TeamCode/src/main/assets/paths/cosmospathing.chor`. Keep this project there. Cosmos writes each generated `.traj` beside the `.chor`, so new trajectories go directly into the FTC Android assets folder without a copy step.

## Generate and run a path

1. Generate and save the path in Cosmos. Its `.traj` file appears in `TeamCode/src/main/assets/paths/`.
2. In VS Code, run **FTC: Install on connected Robot Controller** to rebuild and install the updated app.
3. On the Driver Station, select **Cosmos Timed Path**. During INIT, use the gamepad D-pad left/right to choose a `.traj`, then press START.

The picker lists paths packaged in the installed app. New paths are written into the project immediately; the Hub receives them after the next build and install. To share a generated path with teammates, commit and push its `.traj` file on the `FTCshenanigans` branch.

## How the timed follower works

`MecanumTrajectory` reads the generated sample time, field velocity and heading, plus the wheel geometry and speed limits stored in the Cosmos trajectory. `TestPathing` converts those values to front-left, back-left, back-right and front-right motor powers and applies them against elapsed time. It does not use odometry, a mouse sensor, or feedback correction, so the robot follows the planned timing open-loop and can drift if the real motion differs from the generated path.
