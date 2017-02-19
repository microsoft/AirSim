# Contributing

## Steps

- Make sure your changes conforms to coding style around it. Please read our [short and sweet coding guidelines](coding_guidelines.md).
- [File an issue first](https://github.com/Microsoft/AirSim/issues) so we are aware about change you want to make and possibly guide you.
- Use [usual steps](https://akrabat.com/the-beginners-guide-to-contributing-to-a-github-project/) to make changes just like other GitHub projects.
- Clean compile your changes on Windows and Linux and test basic simulator operations.
- When your pull request is created, you might get prompted to one-time sign [Contributor License Agreement (CLA)](https://en.wikipedia.org/wiki/Contributor_License_Agreement) unless changes are minor. It's very simple and takes less than a minute.
- If your pull request gets a conflict, please resolve it.
- If you pull request shows other changes you need to rebase your branch and try again.
- Watch for any comments on your pull request.
- Please try and limit your changes to small number of files. We need to review every line of your change and we can't reasonably do that if you make requests with huge number of changes.

## Don'ts
- Do not make just cosmetic changes. We will generally reject pull requests with *only* cosmetic changes.
- Do not change code in the deps folder! That's our dependencies and untouchable unless there is very strong reason for it.
- Do not include OS specific header files. Large portions of our library is header-only and we want to keep all our headers clean..
