# Gradle Commands

See this page for a list of WPILIB gradle tasks: https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/gradlew-tasks.html

## Custom Commands

List all tasks

 `./gradlew -q :tasks --all`

Apply spotless formatting

`./gradlew :spotlessApply`

check spotless formatting

`./gradlew :spotlessApply`

# Git Commands

- Check the difference between a file and a previous version

`git diff HEAD^ <file_path>`

- Merge branch without comitting

`git merge --no-ff --no-commit branchX`

- Checkout a file

`git checkout branchX file1`

- Rebase one branch to another

`git rebase branchX`
