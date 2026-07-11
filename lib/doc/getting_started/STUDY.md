# Studies

A study is a project to investigate some specific thing, separately
from the main competition code.

## Create a New Study
WPILib comes with a wizard to start a new project. 

1. Click on the "WPILib" icon and type "New Project".
1. In the New Project wizard, make sure you select `Template > Java > Command Robot`. Then choose a name that describes your project, inside the "studies" folder, and make sure `Desktop Support` is checked. 
1. When it asks what you want to do with the folder, tell it to open in a New Window.


## Create a New Workspace for your Study 
VSCode just opened your single folder, but you actually need the `lib` folder opened as well. VSCode supports the concept of a `workspace` which can include two folders.

To make a workspace file, in your project in vscode, click "add folder to workspace" choosing the lib folder, and then "save workspace to file," choosing your project directory as the location to save it.

This should result in a file at the root level of your project (next to build.gradle) that looks like this:

```json
{
  "folders": [
    {
      "path": "."
    },
    {
      "path": "../../lib"
    }
  ],
    "settings": {
      "java.configuration.updateBuildConfiguration": "automatic",
      "java.server.launchMode": "Standard"
    }
}
```

## Edit build.gradle to include lib code
In your build.gradle file, add this clause just after the "plugins" clause (near the top).

```gradle
sourceSets {
    main {
        java {
            srcDir "../../lib/src/main/java"
        }
    }
}
```

## Copy the `lib` vendordeps to your project
To copy the vendordeps files, find the vendordeps directory in lib, highlight one, say, "NavX.json", and click "copy."

Then find the vendordeps directory in your project (it should contain WPILibNewCommands.json and nothing else).

Highlight the directory name ("vendordeps") and click "paste."

Repeat this process for each of the other vendordeps files.

## Try to Build Again
Always good to make sure everything works :) Run another Build + Deploy and make sure there aren't any errors.

## Send a Pull Request With Your Study
Time to send your code in for review.

1. In vscode, use the git pane to commit.
1. In the Github Desktop, push to your fork.
1. In the github.com webside, create a PR.

