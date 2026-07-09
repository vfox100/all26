# Pi Borg

This is a study of management and deployment of Python to our Raspbery Pi coprocessors.

* [supervisor](https://supervisord.org/index.html) for management.
* gradle for test, packaging, and deployment.

There's a [design doc](https://docs.google.com/document/d/15mUhT1vEGO8y0hBersCINBw1t07pDdcYMedGS_gy2Vk/edit?tab=t.0).

The name "borg" references the thing Google uses for deployment and management,
which works completely differently, but it's a pithy name.

## Demo code

The code here is structured in the same way as our actual pi camera code,
with a script ("runapp.py") at the project root that loads the "zip" file
and runs "main".  There's a directory full of modules ("app") and a
directory full of tests ("tests").

## Formatter

Install the "Black Formatter" extension in vscode, which binds to ctrl-shift-I (on linux,
not sure what the key is on windows, maybe shift-alt-F).

## Testing

Use the "Python" extension in vscode, which should populate the "test" pane once it
is set up.

## Testing in Github Actions

There's a script at the project root that runs the tests ("runtests.py") that
should be run by github.

## Initial setup

Choose Tasks >> build setup >> wrapper from the vscode Gradle pane.  This puts
`gradlew` at the project root.

## Running distribution on the command line

You can say `./gradlew distZip`

## Running distribution from vscode

You can choose Tasks >> distribution >> distZip from the Gradle pane.

## Where is the zip file

One way or the other, the result apears as `build/distributions/app.zip`

## Copying the files

Gradle copies the `runapp.py` file and the dist zip to the destination.

Where should it go exactly?

## Restarting the app

Gradle uses `supervisorctl` to control the app.