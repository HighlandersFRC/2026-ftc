# Cosmos paths in FTC

Open the Cosmos project saved at `TeamCode/src/main/assets/paths/cosmospathing.chor`. Keep this project there. Cosmos writes each generated `.traj` beside the `.chor`, so new trajectories go directly into the FTC Android assets folder without a copy step.

## Generate and run a path

1. Open `cosmospathing.chor` in the Cosmos desktop app.
2. Generate and save the path. Its `.traj` file will appear in `TeamCode/src/main/assets/paths/`.
3. In VS Code, run **FTC: Install on connected Robot Controller** to rebuild and install the updated app.
4. On the Driver Station, select **Cosmos Path Picker**. During INIT, use the gamepad D-pad left/right to choose a `.traj` asset, then press START.

The picker lists paths packaged in the installed app. Generating a path writes it into the project immediately; rebuilding and installing is still needed before the Control Hub receives the new asset.

## FTC path loading

`PurepursuitFollower` reads Cosmos Mecanum `.traj` exports from `trajectory.samples`, using the sample `x`, `y`, and `heading` values with the mouse odometry feedback follower. Cosmos timing and wheel-speed feedforward are not used.
