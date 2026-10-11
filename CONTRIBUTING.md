# Team Contributing Guide

This guide is intended for team members working on the Ball and Plate PID Control project. It defines the expected workflow, development practices, and review standards for collaboration inside the repository.

## Project scope

This repository includes:

- embedded firmware for the ESP32-based control system
- the Python/PyQt desktop GUI
- documentation for the build, testing, and development process

## Working conventions

All team members should:

- keep changes scoped to the relevant subsystem or task
- branch from the current project branch before making changes
- write clear commit messages that explain the intent of the change
- update documentation when setup, behavior, or workflows change
- validate changes before asking for review

## Repository layout

```text
ball-plate-pid-control/
├── firmware/
│   ├── esp32_cam/  # ESP32-CAM firmware
│   └── esp32_s3/   # ESP32-S3 firmware
├── gui/        # Python/PyQt desktop application
├── docs/       # engineering notes and checklists
├── README.md
├── CONTRIBUTING.md
├── .gitignore
└── .vscode/
```

## Local setup

### General requirements

- Git
- Python for GUI development (`3.11` to `3.14` is the current target range)
- ESP-IDF toolchain for firmware development
- VS Code or another suitable editor

### GUI environment

From the repo root:

```powershell
cd gui
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Run the app:

```powershell
$env:PYTHONPATH = "src"
python -m main
```

### Firmware environment

Use the ESP-IDF toolchain in the target-specific directory under `firmware/` (`esp32_cam/` or `esp32_s3/`). Hardware validation should be performed in the smallest practical setup relevant to the change being made.

## Team workflow

1. Create a feature or fix branch for the work.
2. Keep each change focused on a single objective.
3. Commit in small, readable chunks with meaningful messages.
4. Update any related docs when behavior or setup changes.
5. Validate the affected area before requesting review.
6. Open a pull request for discussion and review.

### Recommended branch naming

Use descriptive names such as:

- `feature/ball-tracking-improvements`
- `fix/servo-tuning`
- `docs/gui-setup-update`
- `refactor/control-loop-cleanup`

## Git workflow

Run these commands in PowerShell. Replace the example branch name with a short, descriptive name for your task.

### Clone the repository and create a branch

```powershell
git clone https://github.com/kartoff-an/ball-plate-pid-control.git
cd ball-plate-pid-control
git switch main
git pull
git switch -c feature/describe-your-change
```

If you already have a local clone, update `main` and create your new branch from it:

```powershell
cd path\to\ball-plate-pid-control
git switch main
git pull
git switch -c feature/describe-your-change
```

### Commit and open a pull request

After making and validating your changes, review and commit only the files for your task:

```powershell
git status
git diff
git add path\to\changed-file
git diff --cached
git commit -m "Describe the change"
git push -u origin feature/describe-your-change
```

Open a pull request on GitHub from your branch into `main`. Include a summary of the change and the validation you performed. Keep commits focused, and do not force-push to shared branches.

## Coding expectations

- Prefer readable, maintainable code over clever shortcuts.
- Keep functions and modules focused on a single responsibility.
- Avoid unrelated cleanup in the same change.
- Add comments only when they clarify a non-obvious decision or system detail.
- Document hardware assumptions and calibration constraints when relevant.

## Validation requirements

Before a change is ready for review, the team member should:

- run the relevant GUI tests or targeted validation checks
- verify firmware compilation for the affected target when firmware changes are involved
- confirm that documentation matches the current implementation
- check that the patch does not introduce unrelated regressions

## Pull request standards

Pull requests should include:

- a clear summary of the purpose of the change
- the problem being solved or the improvement being made
- any validation steps performed
- notes about hardware, calibration, or environment constraints when applicable

Keep PRs focused and reviewable. Large cross-cutting changes should be split into smaller, manageable updates.

## Review and handoff

When handing off work:

- state what changed and why
- note any remaining risks or follow-up tasks
- mention the validation that was completed
- call out any hardware or configuration dependencies

This repository is intended for internal team collaboration. The expected standard is clear communication, disciplined review, and reproducible validation.
