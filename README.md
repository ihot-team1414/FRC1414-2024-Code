# 1414 Effect Controls

Try and implement the software so the drivers only have these controls:

## Driver Controller

**Left Stick** Rotate Drivebase (x)
**Right Stick** Translate Drivebase (x,y)
**A, B, X, Y** Lock to cardinal directions
**Start** Reset Gyro
**Right Bumper** Slow Mode
**Left Bumper** Auto Aim (locks direction based on botpose)

## Operator Controller

**A** Start climb / next climb step (use state machine like in 2022)
**B** Intake / Index (should stop at right time)
**X** Score Amp (the entire routine should be 1 button)
**Y** Shoot (check shooter is at right speed, adjust shooter angle, load shooter)

**Back** Outtake through intake
**Start** Reset climb state


## Fully automated
- Shooter should always be at correct angle while in wing
- Shooter should always be spun up in wing
- Indexer stops note when it's at the right spot
