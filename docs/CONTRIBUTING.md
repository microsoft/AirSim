# Contributing

## Quick Start
- Please read our [short and sweet coding guidelines](coding_guidelines.md).
- For big changes such as adding new feature or refactoring, [file an issue first](https://github.com/Microsoft/AirSim/issues).
- Use our [recommended development workflow](dev_workflow.md) to make changes and test it.
- Use [usual steps](https://akrabat.com/the-beginners-guide-to-contributing-to-a-github-project/) to make contributions just like other GitHub projects. If you are not familiar with Git Branch-Rebase-Merge workflow, please [read this first](http://shitalshah.com/p/git-workflow-branch-rebase-squash-merge/).

## Checklist
- Use same style and formatting as rest of code even if it's not your preferred one.
- Change any documentation that goes with code changes.
- Do not include OS specific header files or new 3rd party dependencies.
- Keep your pull request small, ideally under 10 files.
- Make sure you don't include large binary files.
- When adding new includes, make dependency is absolutely necessary.
- Rebase your branch frequently with master (once every 2-3 days is ideal).
- Make sure your code would compile on Windows, Linux and OSX.
