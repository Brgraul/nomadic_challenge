- [x] Read poteto-mode Principles and route to figure-it-out for coordinated document revision.
- [x] Phase A. Frame. Preserve all 16 original findings and rank by dependency and impact.
- [x] Phase B. Design workflow. Capture baseline, edit in ranked order, keep one writer, use independent final readers.
- [x] Phase C. Run loop. Make each ranked change and inspect its actual section before the next change.
- [x] Phase D. Keep audit trail. Record each ranked change and its evidence in decisions.tsv.
- [x] Phase E. Verify and hand back. Audit every original finding against final prose, types, diagrams, examples, and acceptance scenarios.
- [x] Independent state-machine review.
- [x] Independent estimation and optimization review.
- [x] Address all accepted review findings and rerun final audit.
- [x] Produce ranked requirement-by-requirement validation report.

Throughput checkpoint:
- Blocking first steps. Inspect current file, capture its baseline before editing, and use the earlier chat review; materialize its checklist for the final audit.
- Independent workstreams. Two read-only reviewers inspect final artifact after serial edits.
- Shared mutable state. Only the root edits planner-design.md; reviewers return findings.
- Smallest safe decomposition. One writer preserves consistent shared terminology and changes contracts in dependency order.

Definition of done. Every original finding has an explicit resolution and final-artifact evidence; the cross-cutting review finds no unresolved contradictions; diagrams, type sketches, examples and acceptance scenarios agree. This verifies the design document, not the unimplemented application.
