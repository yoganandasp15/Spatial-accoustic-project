# Use this handoff pack with Gemini CLI

This package gives Gemini CLI persistent project context and includes the original proposal, available evidence ZIPs, Stage 1B summary CSVs, and hardware photos.

## 1. Extract the ZIP

Extract the contents into one project folder, for example:

`D:\ECE\Spatial_Multimodal_EdgeAI\`

Make sure `GEMINI.md` is directly in that folder, not buried inside another nested folder.

## 2. Open a terminal in that folder

In Windows File Explorer, open the extracted folder, click the address bar, type `powershell`, and press Enter. This opens PowerShell at that folder.

## 3. Start Gemini CLI

Run:

```powershell
gemini
```

Gemini CLI automatically discovers `GEMINI.md` context files hierarchically from the current directory and parent directories. Project-level `GEMINI.md` is the right place for shared project instructions/context. See official documentation:

- https://geminicli.com/docs/cli/gemini-md/
- https://geminicli.com/docs/cli/tutorials/memory-management/

## 4. Check that Gemini loaded the project context

Inside Gemini CLI, run:

```text
/memory show
```

You should see the content from the project `GEMINI.md` and its imported context files. If you edited `GEMINI.md` after starting the session, run `/memory refresh` (or `/memory reload` if that is the command offered by your installed CLI version), then `/memory show` again.

## 5. Review Master Project Digest & Multi-Agent Protocol

Read `GPT_REVIEW_PACK.md` for a complete technical digest of derivations, algorithms, and empirical benchmark findings.
Review `AGENTS.md` and `docs/AGENT_SYNCHRONIZATION_LOG.md` for the multi-agent operating protocol and activity history.

## 6. Add your working files

When you later create Colab notebooks, Python scripts, circuit diagrams, or new output archives, save copies under this project folder (for example `stage1_...`, `stage2_...`, `evidence/`, and `hardware_photos/`). AI agents can inspect local files in the working directory.

Do not delete the original uploaded evidence ZIPs. Keep a separate copy of new results with a date/version in the filename.

## 7. Important

Project documentation and evidence archives are included so any AI or human evaluator can inspect the source code, unit tests, and outputs directly.
