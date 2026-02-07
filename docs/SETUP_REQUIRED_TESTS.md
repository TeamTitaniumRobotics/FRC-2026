# Setting Up Required Tests for Pull Requests

This guide shows how to configure your repository so that all tests must pass before code can be merged.

## Step 1: Enable GitHub Actions

1. Go to your repository on GitHub: `https://github.com/TeamTitaniumRobotics/FRC-2026`
2. Click on **Actions** tab
3. If prompted, click **I understand my workflows, go ahead and enable them**

The CI workflow (`.github/workflows/ci.yml`) will now run automatically on pull requests.

## Step 2: Configure Branch Protection Rules

### For the `main` branch:

1. Go to **Settings** → **Branches** (in repository settings)
2. Click **Add branch protection rule**
3. Branch name pattern: `main`
4. Enable these settings:

   ✅ **Require a pull request before merging**
   - Require approvals: 1 (optional, recommended for teams)
   - Dismiss stale pull request approvals when new commits are pushed (recommended)

   ✅ **Require status checks to pass before merging**
   - **This is the critical one!**
   - Search for: `CI - Build and Test`
   - Check the box next to it
   - ✅ Require branches to be up to date before merging

   ✅ **Do not allow bypassing the above settings** (optional but recommended)

5. Click **Create** or **Save changes**

### For the `develop` branch (if you use one):

Repeat the same steps for branch pattern: `develop`

## Step 3: Verify Setup

1. Create a test branch: `git checkout -b test-ci`
2. Make a small change (e.g., add a comment somewhere)
3. Commit and push:
   ```bash
   git add .
   git commit -m "Test CI setup"
   git push -u origin test-ci
   ```
4. Create a pull request on GitHub
5. You should see:
   - 🟡 "CI - Build and Test" workflow starts running
   - After a few minutes: ✅ "All checks have passed" (green)
   - **Merge button** should only be enabled after tests pass

## Step 4: What Happens Now?

### On every pull request:
1. **Automatic build** - Code is compiled
2. **Run all tests** - Both `RobotTest` and `TurretTest` execute
3. **Generate reports** - Test results and coverage posted as PR comment
4. **Block merge if failing** - Cannot merge if tests fail ❌

### Team members will see:
- ✅ Green checkmark if all tests pass
- ❌ Red X if any test fails
- 📊 Detailed test results in PR comments
- 📈 Code coverage percentage

## Advanced Options (Optional)

### Require code coverage threshold:
In `.github/workflows/ci.yml`, add after the test step:
```yaml
- name: Check coverage threshold
  run: ./gradlew jacocoTestCoverageVerification
```

This will fail the build if coverage drops below 30%.

### Require code review:
In branch protection rules:
- Set "Require approvals" to 1 or 2
- Only designated reviewers can approve PRs

### Prevent force pushes:
In branch protection rules:
- ✅ Do not allow force pushes
- ✅ Do not allow deletions

### Auto-merge when tests pass:
GitHub has an "Enable auto-merge" button on PRs. When enabled:
1. Tests run automatically
2. If tests pass AND approvals met → auto-merge
3. If tests fail → auto-merge is cancelled

## Testing the Setup

### Test Case 1: Passing Tests
```bash
# Make a small change
echo "// Test comment" >> src/main/java/org/teamtitanium/Robot.java
git add .
git commit -m "Add comment"
git push
# Create PR - should see ✅ green checks
```

### Test Case 2: Failing Tests
```bash
# Break a test intentionally
# Edit TurretTest.java and change an assertion to fail
git add .
git commit -m "Break test (intentional)"
git push
# Create PR - should see ❌ red X and merge blocked
```

### Test Case 3: Build Error
```bash
# Add syntax error
echo "invalid java syntax here" >> src/main/java/org/teamtitanium/Robot.java
git add .
git commit -m "Break build (intentional)"
git push
# Create PR - should see ❌ build failure
```

## Troubleshooting

### Problem: "CI - Build and Test" doesn't appear in status checks
**Solution:**
1. Make sure the workflow file exists: `.github/workflows/ci.yml`
2. Workflow must run at least once before it appears
3. Push a commit to trigger it, then configure branch protection

### Problem: Tests pass locally but fail in CI
**Possible causes:**
- CI uses Linux, you might be on Windows/Mac (path differences)
- Missing dependencies in CI environment
- Different Java version

**Solution:** Check the workflow logs in GitHub Actions tab

### Problem: "No test report files were found"
**Solution:**
- Verify tests are actually running: `./gradlew test`
- Check that `build/test-results/` directory is created
- Ensure `test` task completes successfully

### Problem: Workflow permission errors
**Solution:**
1. Go to **Settings** → **Actions** → **General**
2. Under "Workflow permissions":
   - Select "Read and write permissions"
   - ✅ Allow GitHub Actions to create and approve pull requests
3. Save

## Monitoring Test Health

### View test history:
1. Go to **Actions** tab
2. Click on any workflow run
3. View detailed logs and test results

### View test trends:
- Install the [Test Reporter](https://github.com/marketplace/actions/test-reporter) GitHub Action
- Adds charts showing test pass/fail trends over time

### Get notifications:
- Watch the repository (⭐ Watch → All Activity)
- Notifications when:
  - PR checks fail
  - Someone force-pushes (if protected)
  - New PRs are opened

## Best Practices

1. **Run tests locally before pushing:**
   ```bash
   ./gradlew test
   ```

2. **Don't bypass branch protection** (even if you're an admin)
   - This defeats the purpose!
   - If you must, document why in the commit message

3. **Fix failing tests immediately**
   - Broken tests block everyone
   - Don't let them linger

4. **Keep tests fast**
   - Current tests run in ~10 seconds
   - Slow tests = frustrated team members
   - Target: < 30 seconds for all tests

5. **Review test failures in PRs**
   - Don't just re-run until they pass
   - Investigate root cause
   - Fix flaky tests

## Summary Checklist

- [ ] `.github/workflows/ci.yml` file exists
- [ ] JUnit tests in `src/test/java/` directory
- [ ] `build.gradle` has JaCoCo plugin configured
- [ ] GitHub Actions enabled for repository
- [ ] Branch protection rule created for `main`
- [ ] "Require status checks" enabled
- [ ] "CI - Build and Test" selected as required check
- [ ] Tested with a sample PR
- [ ] Team members know tests must pass before merge

Once all checkboxes are complete, your repository is protected! 🎉

## Questions?

- Check the GitHub Actions logs for detailed error messages
- Review the test output in `build/reports/tests/test/index.html`
- Ask programming leads for help with CI/CD setup
