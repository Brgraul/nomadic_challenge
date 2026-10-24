# Planner design revision evidence

This directory preserves the original document, the 16-finding checklist, sequential edit records, reconciliation records, numerical rank evidence and structural document checker for the design revision. See [the validation report](../../docs/planner-design-validation.md) for the ranking and requirement-by-requirement conclusions.

`original.md` is the pre-edit baseline. `step01.py` through `step16.py`, `edit.py` and reconciliation scripts are historical mutation records, not commands to run against the final document. Their original absolute temporary paths are preserved. The append-only `decisions.tsv` records changes and later audit corrections; later correction rows supersede weak evidence or timestamp claims rather than rewriting them. Historical `/private/tmp/planner-design-revision` references have their preserved equivalents here.

`requirements.md` was materialized from the earlier chat review after the serial edit pass and used for final cross-validation. It does not prove pre-edit checklist capture. The original review and pre-edit ranking remain in this chat.

Only the read-only document/rank checks are intended to be rerun. The rank checker refreshes `rank-result.json` as an evidence artifact. Run from the repository root:

```sh
.venv/bin/python .audit/planner-design/check_document.py
```

Document validation does not implement or test the planned application. Python snippets are type sketches, not a complete module. Runtime acceptance scenarios in the design are future implementation requirements.

## Modular layout evidence

`pre-split.md` preserves the fully reviewed monolithic document. `split-preservation-result.json` captures the lossless 42-section relocation before the user's timing amendment. `split-manifest.json` maps those sections to current chapters and identifies intentional amendments. Historical `check_split.py` checks that earlier relocation checkpoint and is not the final current-layout verifier; use `check_document.py` for the current chapters, references and fresh rank calculation. `split_layout.py`, `realtime_amendment.py` and `link_chapters.py` are historical mutation records, not commands to rerun on the final layout.

The new front file is a selective-loading index. Chapter-level contracts are authoritative. Old monolithic evidence anchors in historical trail rows can be located through `split-manifest.json` or read in `pre-split.md`.
