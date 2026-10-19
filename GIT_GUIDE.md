# Git Workflow

Project folder: `AAA_group_assignment`

Remote repository: `https://github.com/Atlas13916169797/5560Group-project.git`

## Before editing

Open PowerShell in the project folder. When the working tree is clean, download the latest shared changes:

```powershell
git pull --ff-only
```

## After editing

Save your files, inspect the changes, create a commit and push it:

```powershell
git status
git add --all
git commit -m "Update project files"
git push
```

Alternatively, run the included helper from PowerShell:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\push.ps1 -Message "Update project files"
```

The execution-policy override applies only to this command. The helper stages all project changes, including file moves and deletions, commits them and pushes the current branch. It stops on errors and never force-pushes. Review `git status` before running it.

GitHub Credential Manager may open the browser for the first authentication. Complete the sign-in there; do not enter passwords or tokens in chat. Credentials are managed by Windows Credential Manager.

If Git is not recognized immediately after installation, reopen PowerShell or restart the editor. The helper also checks the standard Git installation paths.

`.gitignore` excludes editor lock files and LaTeX build intermediates. Aircraft `.p` files, initial-condition and result `.mat` files, PDF reports and figures remain eligible for tracking. Ignore rules do not automatically untrack files already committed in the remote history.

Git does not push automatically when a file is saved. Use the commands or helper above for each update. Fetch shared changes before editing to reduce conflicting work.
