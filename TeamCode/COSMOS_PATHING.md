# Cosmos pathing for FTC

The `TeamCode` folder on this branch comes from the `ObjectDetection` branch. It includes the mecanum drive, odometry, and existing Pure Pursuit follower. The Cosmos export `src/main/assets/paths/MecanumSmoke.traj` is synchronized from the open Cosmos project.

`TestPathing` loads `MecanumSmoke.traj` directly. The follower accepts Cosmos JSON under `trajectory.samples` when `trajectory.sampleType` is `Mecanum`, and reads `x`, `y`, and `heading` from each sample. It continues to support the older `sampled_points` / `.polarpath` files too.

## Generate and sync another Cosmos path

1. In Cosmos, generate and save the path so the `.traj` export is written beside the `.chor` project.
2. Copy the generated `.traj` into `TeamCode/src/main/assets/paths/`.
3. Update the filename in `TestPathing` to match the exported file.
4. Build and install the Robot Controller app from VS Code.

Cosmos exports SI coordinates with its lower-left field origin and counterclockwise headings. The existing FTC follower resets odometry to the path's first point and uses its own feedback controller; this integration reads the Cosmos path geometry and heading samples, not Cosmos time-based wheel feedforward.
