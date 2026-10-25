# P2 standing orders (one writer: the implement_p2 owner session chain)

Implement docs/three-method-outage-plan.md P2 in this worktree (branch three-method-p2,
base 793596d = P1). Design contract: .audit/p2-design.md. One coherent change, commit
locally, no PR publishing, no merge, no deploy, no scheduled automation. Real evidence
only; mark unsupported verification honestly. The operator merges.

2026-10-25 recovery note: the original /tmp/nomadic-three-method-p2 checkout and all /tmp
receipts were wiped by the OS mid-unit. P1 survives on main-repo branch three-method-p1.
This durable worktree (sibling of the main repo) replaces the /tmp checkout. Step 1's
uncommitted sensing changes were lost with it and are redone here. Lesson: no work lives
under /tmp; receipts mirror under .audit/.

Autopilot-stack steps (verbatim), adapted to local-only by standing orders:

1. **Run Autopilot-full's owners, minus the landing.** Owners report STACK-READY in place of
   merge-ready. They never rebase onto another branch, trunk included, and never retarget,
   merge, close a PR, or arm auto-merge, because the root owns the chain's topology and the
   operator owns the landing. When an item depends on another, its owner branches from that
   item's branch, not from trunk.
   adapted: no PRs exist; the chain is local commits on three-method-p2, based on 793596d
   (P1). Owner reports STACK-READY when unit+live+perf boxes are checked.
2. **Give the topology one writer.** Owners push only their own branches and report each tip
   and its intended parent. Only you rebase or retarget a link, because two writers on one
   chain undo each other.
   adapted: one owner session chain writes this worktree; the root records the head SHA.
   skip: remote pushes and `--force-with-lease` mechanics; no remote by standing orders.
3. **Absorb drift at the root.** Until landing starts, apply Autopilot-full step 5's test to
   each link and rebase the chain bottom-up where it calls for one.
   skip: trunk cannot move under us mid-unit; P1 is a local commit and the chain has one link
   in flight. Recorded base 793596d, head recorded at verdict.
4. **Deliver the chain, then keep your hands off it.** Each link carries its verdict in its
   PR. Once landing starts, nobody writes to the chain except the post-merge retarget that
   `playbooks/shipping.md` sanctions.
   adapted: the link carries its verdict in .audit/three-method-p2.tsv and the commit
   message; landing is the operator's.

Autopilot-full steps that carry over (verbatim where applied):

1. The operator's gates come first. (Standing orders recorded 2026-10-24 in the program
   store; program size restated to the operator before the first owner started.)
2. Spawn one owner per PR... skip: PR opening; local commit instead. Decision trail runs in
   .audit/three-method-p2.tsv. Assume the owner's session dies: every phase ends with the
   trail row and a report a replacement can resume from.
3. Run owners in parallel off trunk, never stacked. skip: plan requires a dependent P1..P4
   chain and serialized schema edits; exactly one writer.
4. Swarm-verify once. A full swarm runs once, at the code-ready head. Fan out independent
   verifiers at the code-ready SHA per the swarm skill; they re-run the gates, prove the
   load-bearing behavior live on the real surface, and audit the diff. Send every proven
   finding to the owner in one fix-forward; two fix-forwards per link is the ceiling.
   Screenshots stay a plan review-gate item where computer-use is blocked; record honestly.
6. Run the root layer. skip: hourly audit tick; the operator excluded scheduled automation
   for this bounded implementation. The root session probes each phase delegate by its side
   effects between phases instead.
7. Stand down at once on the operator's stop.

- [x] Ground (plan, verdicts, baseline receipts; baseline receipts lost in /tmp wipe,
      their recorded values live in the main repo's .audit/three-method-outage.tsv)
- [x] Sketch (.audit/p2-design.md)
- [x] Phase A step 1: sensing request contracts + compile_requests (redone after /tmp wipe)
- [x] Phase A step 2: world itinerary/energy execution
- [x] Phase B: routes step validation; needs request-lifecycle refresh; scoring + planning
- [x] Phase C: proposals/decisions lifecycle; agents/scripted/coordinator; sim wiring, knowledge/assessment
- [x] Phase D: UI + e2e + driver lanes; full suite + ruff; local commit with verdict
- [x] Swarm-verify once at the code-ready head (PASS-WITH-FINDINGS, 0 blocking)
- [x] Fix-forwards (ceiling two) then STACK-READY report
