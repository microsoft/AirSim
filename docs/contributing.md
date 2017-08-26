# Contributing

## How To

### Prepare

- Please read our [short and sweet coding guidelines](coding_guidelines.md).
- For big changes such as adding new feature or refactoring, [file an issue first](https://github.com/Microsoft/AirSim/issues). We should talk!
- Use [usual steps](https://akrabat.com/the-beginners-guide-to-contributing-to-a-github-project/) to make contributions just like other GitHub projects. If you are not familiar with Git Branch-Rebase-Merge workflow, please [read this first](http://shitalshah.com/p/git-workflow-branch-rebase-squash-merge/).

### Submit
- Make code changes in your fork and new branch
- Clean compile your changes on Windows or Linux and test basic simulator operations.
- When your pull request is created, you might get prompted to one-time sign [Contributor License Agreement (CLA)](https://en.wikipedia.org/wiki/Contributor_License_Agreement) unless changes are minor. It's very simple and takes less than a minute.
- If your pull request gets a conflict, please resolve it.
- If you pull request shows other changes you need to rebase your branch and try again.
- Watch for any comments on your pull request.


## Don'ts

- Do not make just cosmetic changes. We will generally reject pull requests with *only* cosmetic changes.
- Do not change code in the deps folder! That's our dependencies and untouchable unless there is very strong reason for it.
- Do not include OS specific header files. Large portions of our library is header-only and we want to keep all our headers clean.
- Do not make changes to very large number of files. We need to review every line of your change and we can't reasonably do that if you make requests with huge number of changes.
