# Repository Guide

Shared MATLAB and Simulink files for the project.

**Every team member has their own branch.** You work on yours, nobody else touches it, and finished work gets merged into `main` via a Pull Request. `main` is the clean, working version everyone builds from.

**Never used git before? Read the whole thing once. It takes about ten minutes and saves a lot of pain later.**

---

## What git actually does

Think of it as a shared folder with an undo history and a rulebook.

- The **remote** is the copy on GitHub. It's the single source of truth.
- Your **local** copy is the folder on your laptop. You edit here.
- **Pull** = download other people's changes into your local copy.
- **Commit** = save a snapshot of your work locally, with a note describing it.
- **Push** = upload your commits to the remote so everyone else can get them.
- A **branch** is a parallel version of the files. You have your own, so you can work without overwriting anyone else or being overwritten.

Nothing you do locally affects anyone else until you push.

---

## One-time setup

**1. Install git**

- Windows: https://git-scm.com/download/win (accept all defaults)
- macOS: open Terminal, type `git --version`, and it will offer to install

**2. Tell git who you are**

Open a terminal (Windows: Git Bash, macOS: Terminal) and run:

```bash
git config --global user.name "Your Name"
git config --global user.email "your.student.email@uts.edu.au"
```

**3. Get the repo onto your machine**

Navigate to where you want the folder to live, then:

```bash
cd Documents
git clone <REPO-URL-HERE>
cd <REPO-NAME>
```

You now have the whole repository, including its full history.

> **Prefer buttons to typing?** [GitHub Desktop](https://desktop.github.com/) does everything below with a GUI, and MATLAB has git built into the Current Folder browser (right-click → Source Control). The concepts are identical — same words, same order. The command line is documented here because it works everywhere and is easier to get help with.

---

## Rule number one: pull first

**Before you open MATLAB. Every single time. No exceptions.**

```bash
git pull
```

This is the most important habit in this document, and it's the one people skip.

Here's why it matters. Git is fine with you being behind — it will happily let you spend three hours building on an old version of the files. It only complains later, when you try to push and your work and everyone else's have drifted apart. At that point untangling it is a genuine headache: an afternoon of merge conflicts, confusing error messages, and someone in the group chat asking why the model won't build.

None of that is dangerous. Nothing gets permanently destroyed. It's just tedious, avoidable, and it always seems to happen the week before a deadline.

`git pull` takes two seconds and prevents essentially all of it.

**Pull at the start of every session. Pull again before you push. Pull if you've been away from the repo for a few days. There is no such thing as pulling too often** — if there's nothing new, it does nothing.

---

## The everyday loop

Four commands. This is 95% of what you'll ever do.

```bash
git pull                          # 1. FIRST. always. see above.
                                  # 2. do your work in MATLAB, save your files
git add .                         # 3. mark all your changes to be saved
git commit -m "describe what you did"
git push                          # 4. send it up so the team can see it
```

**Write real commit messages.** `"fixed slip ratio calc in tyre model"` is useful in three weeks. `"update"` and `"asdf"` are not.

---

## Your branch

Everyone gets one branch with their name on it. Yours is where all your work lives. Use your first name, lowercase, no spaces — `talis`, `priya`, `jordan`.

**Check where you are.** The branch with the `*` next to it is the one you're currently on:

```bash
git branch -a
```

**Move onto your branch:**

```bash
git checkout your-name
```

If it doesn't exist yet, create it off `main` and push it up once so the remote knows about it:

```bash
git checkout main
git pull
git checkout -b your-name
git push -u origin your-name
```

After that first push, plain `git push` works from then on.

**Stay on your branch.** Once you're on it, you stay on it. Everything you commit and push goes there and affects nobody else. You do not need to switch branches during normal work, and you should never commit directly to `main`.

Moving work between your branch and `main` is the next section, and it's worth reading properly.

---

## Moving work between `main` and your branch

`main` is the shared, working version of the project. Your branch is your workspace. Work travels between them in two directions, and they happen at different times for different reasons.

### Direction 1: `main` → your branch (bringing others' work in)

```bash
git pull origin main
```

This pulls everyone else's merged work into your branch. Run it from your branch — you don't need to switch to `main` first.

**When to do it:**

- Every few days, as routine maintenance
- Before you start a new chunk of work
- Before you open a Pull Request (see below) — this is the important one
- Whenever someone announces in the group chat that they've merged something

Your branch drifts further from `main` every day you leave this. Doing it often means each update is small and usually silent. Leaving it for a month means one large, tangled merge — same headache described in rule number one, just a bigger version of it.

### Direction 2: your branch → `main` (sharing your work out)

This happens through a **Pull Request** on GitHub, not on your machine. Never merge into `main` yourself from the command line.

```bash
git pull origin main    # 1. bring main up to date in your branch first
git push                # 2. make sure everything is pushed
```

Then in the browser: open the repo, click **Compare & pull request**, write a sentence or two on what you did, and create it. Someone else reviews it and merges it. Your branch stays alive afterwards — keep working on it.

**When to do it — the three triggers:**

1. **Something is finished.** A subsystem, a script, a fix. It's done, it's tested, it's not going to change again this week.
2. **Something works.** Not necessarily finished, but at a stable point where it runs without errors and doesn't break anything else. Working-and-partial belongs in `main` more than perfect-and-hidden does.
3. **Someone else needs it.** If another person is blocked on your work, or is about to build on top of it, get it into `main` so they're building on the real thing rather than a copy you emailed them.

**When *not* to do it:** if it's half-written, doesn't run, or you're mid-experiment. That's exactly what your own branch is for — commit and push it there as much as you like, it just doesn't go to `main` yet.

The general rule: **little and often beats one enormous merge at the end.** Small Pull Requests are quick to review and rarely conflict. A month of unmerged work is a bad afternoon for whoever reviews it, and usually for you too.

---

## MATLAB and Simulink specifics — read this part

**`.m` files are text.** Git handles them beautifully. Two people can edit the same `.m` file in different places and git will combine both sets of changes automatically.

**`.slx` and `.mdl` files are binary.** Git cannot combine two people's edits to the same model. If you and someone else both change `traction_controller.slx`, one of you will lose your work.

> **The rule: message the group chat before you start editing a shared model, and again when you've pushed it.** Low-tech, but it's the only thing that reliably works.

To see what changed between two versions of a model, use MATLAB's own tool rather than git's — the Comparison Report under the Compare button, or `visdiff('file1.slx','file2.slx')` in the command window.

**Don't commit generated junk.** Build artefacts, autosaves and cache folders will clog the repo and cause pointless conflicts. The `.gitignore` file in this repo already handles the usual suspects (`slprj/`, `*.asv`, `codegen/`, `*.slxc`). If you notice something generated sneaking in, add it there.

---

## When something goes wrong

Nothing here is unrecoverable. Git keeps everything. If you're stuck, **stop and ask before typing commands you found on Stack Overflow** — that's the one way to actually lose work.

**"Updates were rejected... non-fast-forward"**

The classic didn't-pull-first error. The remote has changes you don't. Fix:

```bash
git pull
git push
```

**"CONFLICT (content): Merge conflict in ..."**

You and someone else edited the same lines of the same text file. Git has marked both versions inside the file like this:

```
<<<<<<< HEAD
your version
=======
their version
>>>>>>> main
```

Open the file, delete the `<<<<<<<`, `=======` and `>>>>>>>` lines, and leave behind the code you actually want (sometimes that's a bit of both). Then:

```bash
git add .
git commit -m "resolve merge conflict"
git push
```

If it's an `.slx` file, don't try to edit it by hand — talk to whoever else touched it and decide whose version survives.

**"I want to throw away my local changes and start again from what's on the remote"**

Destructive — your uncommitted edits are gone for good:

```bash
git checkout -- .
git pull
```

**"What have I even changed?"**

```bash
git status      # which files are modified
git diff        # the actual line-by-line changes
git log --oneline   # recent commit history
```

---

## Cheat sheet

| Goal | Command |
|---|---|
| **Get the latest changes (do this first)** | **`git pull`** |
| Get the repo for the first time | `git clone <url>` |
| See what you've changed | `git status` |
| Save a snapshot | `git add .` then `git commit -m "message"` |
| Upload your work | `git push` |
| Check which branch you're on | `git branch -a` |
| Move onto your branch | `git checkout your-name` |
| Create your branch (once) | `git checkout -b your-name` |
| Push your branch the first time | `git push -u origin your-name` |
| Bring `main`'s updates into your branch | `git pull origin main` |
| Share your work into `main` | Push, then open a Pull Request on GitHub |

---

## Habits worth having

1. **Pull before you start. Pull before you push.** If you only remember one line of this document, make it this one.
2. Commit small and often. One commit per logical change beats one giant commit at the end of the week.
3. Stay on your own branch. Never commit directly to `main`.
4. Open a Pull Request whenever something is finished, working, or needed by someone else. Small and frequent beats one huge merge at the end.
5. Announce it in the group chat before editing a shared `.slx`.
6. Never commit anything you wouldn't want the whole team — or a design judge — to read.