# Contributing

## Steps

1. [File an issue first](https://github.com/Microsoft/AirSim/issues) so we are aware about change you want to make and possibly guide you.
2. Use [usual steps](https://akrabat.com/the-beginners-guide-to-contributing-to-a-github-project/) to make changes just like other GitHub projects.
3. Clean compile your changes on Windows or Linux and test basic simulator operations.
4. When your pull request is created, you might get prompted to one-time sign [Contributer License Agreement (CLA)](https://en.wikipedia.org/wiki/Contributor_License_Agreement) unless changes are minor. It's very simple and takes less than a minute.
5. Watch for any comments on your pull request.
 
## Don'ts
- Do not make just cosmatic changes. We will generally reject pull requests with *only* cosmatic changes.
- Limit your changes to small number of files. We need to review every line of your change and we can't reasonably do that if you make requests with 10s of files.
- Do not change code in the deps folder! That's our dependencies and untouchable unless there is very strong reason for it.
- Make sure your changes conforms to style around it. Take a look at our [short and sweet guidelines](coding_guidelines.md).
- Do not include OS specific header files. Large portions of our library is header-only and we want to keep all our headers clean..

# Coding Style
We have the [sweet and short coding guideline](coding_guidelines.md) document you might have yet seen.
