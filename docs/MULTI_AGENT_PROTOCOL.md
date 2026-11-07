# 🤖 Multi-Agent Collaborative Operating Protocol

> **Target Repository:** `yoganandasp15/Spatial-accoustic-project`  
> **Repository URL:** `https://github.com/yoganandasp15/Spatial-accoustic-project`  
> **Rule for All Agents:** Every autonomous agent (Antigravity, Gemini, ChatGPT, Claude, Codex, Aider) operating on this repository MUST strictly follow the protocol below.

---

## 🔄 1. Pre-Execution Synchronization Invariant (Pull First)
Before inspecting files, formulating hypotheses, or making code edits:
```bash
git pull origin main
```
Immediately read `docs/AGENT_SYNCHRONIZATION_LOG.md` and the most recent file in `docs/daily_logs/` to discover what concurrent agents, subagents, or external reviewers (such as ChatGPT) have implemented or reported.

---

## 📝 2. Agent Attribution & Logging Contract
Whenever you investigate, modify code, tune firmware, or generate evidence:
1. Append an entry to the top of the ledger in `docs/AGENT_SYNCHRONIZATION_LOG.md` using the standard format:
   ```markdown
   | <YYYY-MM-DD HH:mm:ss IST> | <Agent / Conversation ID> | <Subsystem / File> | <Summary of Action & Findings> | <Commit SHA> | <PASS / IN-PROGRESS> |
   ```
2. If your work updates project milestones or addresses external feedback, update `docs/PROJECT_STATUS.md` and `GPT_REVIEW_PACK.md` accordingly.

---

## 🚀 3. Attributed Commit & Push Protocol
Every commit MUST identify the agent and execution timestamp:
```bash
git add -A
git commit -m "<type>(<scope>): [Agent <short-id>] [<YYYY-MM-DD HH:mm>] <clear concise summary>"
git push origin main
```

**Commit Format Examples:**
* `fix(firmware): [Agent 1fbd76d3] [2026-11-07 20:15] unpack INMP441 audio with >>16 standard MSB shift`
* `feat(gui): [Agent a4c98e12] [2026-11-07 20:30] add live evidence mode HUD banner and ISO-inspired severity proxy`
* `docs(audit): [Agent 1fbd76d3] [2026-11-07 20:45] reconcile ChatGPT peer review points into evidence matrix`

---

## 🛡️ 4. Core Engineering Invariants
* **Hardware Pinout Invariant (Classic ESP32 COM8 @ 921,600 baud):**
  * I2S Master SCK = GPIO 18, WS = GPIO 21.
  * Pair A Audio = GPIO 19 (I2S0 SD). Pair B Audio = GPIO 23 (I2S1 SD).
  * ADXL345 I2C = SDA TX2 (GPIO 17), SCL RX2 (GPIO 16).
  * FC-03 Tachometer ISR = D4 (GPIO 4).
* **Audio Bit-Shift Invariant:** INMP441 24-bit audio in 32-bit slot must be shifted `>> 16` directly into signed `int16_t` (never `>> 14`).
* **Evidence Separation Invariant:**
  * Machine Fault Ground Truth: MAFAULDA (10g unbalance $1\times$ surge) & CWRU datasets.
  * Acoustic Localization Ground Truth: Stage 2 25-point off-grid simulation ($0.34^\circ$ median error).
  * Hardware Prototype Ground Truth: Classic ESP32 streaming on COM8 @ 921,600 baud.
