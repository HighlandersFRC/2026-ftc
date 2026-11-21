# Deploying from VS Code

This workspace includes two VS Code tasks for the FTC Android project:

- **FTC: Build Robot Controller APK** builds the debug APK.
- **FTC: Install on connected Robot Controller** builds and installs that APK on a Robot Controller visible to ADB.

Open **Terminal → Run Task…** and select a task. Before installing, connect the Control Hub to the computer and make sure it appears in `adb devices`.

The tasks use the Android Studio Java runtime and the default Windows Android SDK location. If Android Studio or the SDK is installed elsewhere, update `JAVA_HOME` or `ANDROID_HOME` in `.vscode/tasks.json` to match your machine.

VS Code will recommend the Java Extension Pack and Gradle for Java when the workspace opens.
