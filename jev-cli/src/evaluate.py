"""
Crew-dispatch decision logic.

Gemini returns four probabilities; code computes dispatch_priority and Decision:

  priority = P(vegetation_contact) × P(impact)

  priority >= 0.64  AND  evidence_count >= 2  →  DISPATCH_CREW
  priority >= 0.16                             →  MEASURE_MORE
  priority <  0.16                             →  DEPRIORITIZE

Thresholds: 0.8 × 0.8 = 0.64 maps to the "both signals strong" case from spec.
"""

import os
from google import genai
from google.genai import types
from pydantic import BaseModel, Field

from .types import Decision, EvidenceScores, EvaluationResult, SegmentHypothesis
from .questions import build_prompt

DISPATCH_PRIORITY_THRESHOLD = 0.64   # ~0.8 × 0.8
DEPRIORITIZE_THRESHOLD = 0.16        # ~0.4 × 0.4
EVIDENCE_THRESHOLD = 0.60
MIN_EVIDENCE_FOR_DISPATCH = 2


class EvidenceOutput(BaseModel):
    vegetation_contact_probability: float = Field(
        description="0.0–1.0. P(tree is touching or about to contact the conductor)."
    )
    impact_probability: float = Field(
        description="0.0–1.0. P(segment failure causes significant customer/infrastructure impact)."
    )
    report_credibility: float = Field(
        description="0.0–1.0. P(reports credibly implicate this specific segment)."
    )
    innocent_explanation: float = Field(
        description="0.0–1.0. P(benign explanation fully accounts for all observations)."
    )


def _decide(priority: float, evidence_count: int) -> Decision:
    if priority >= DISPATCH_PRIORITY_THRESHOLD and evidence_count >= MIN_EVIDENCE_FOR_DISPATCH:
        return Decision.DISPATCH_CREW
    if priority < DEPRIORITIZE_THRESHOLD:
        return Decision.DEPRIORITIZE
    return Decision.MEASURE_MORE


def _reasoning(seg, scores, veg_p, imp_p, priority, evidence_count, decision) -> str:
    lines = [
        f"Segment {seg}:",
        f"  P(tree_contact)={veg_p:.2f}  P(impact)={imp_p:.2f}  → priority={priority:.3f}  evidence_count={evidence_count}",
        f"  report_credibility={scores.report_credibility:.2f}  innocent_explanation={scores.innocent_explanation:.2f}",
        f"  → {decision.value}",
    ]
    if decision == Decision.DISPATCH_CREW:
        lines.append("  Action: dispatch operator crew for physical inspection.")
    elif decision == Decision.MEASURE_MORE:
        lines.append("  Action: send drone for sensor readings before committing a crew.")
    else:
        lines.append("  Action: deprioritize — reallocate drone capacity to higher-priority segments.")
    return "\n".join(lines)


def evaluate(hypothesis: SegmentHypothesis) -> EvaluationResult:
    prompt = build_prompt(hypothesis)

    client = genai.Client(api_key=os.environ["GEMINI_API_KEY"])
    response = client.models.generate_content(
        model="gemini-3.8-flash",
        contents=prompt,
        config=types.GenerateContentConfig(
            response_mime_type="application/json",
            response_schema=EvidenceOutput,
            temperature=0.0,
        ),
    )

    out = EvidenceOutput.model_validate_json(response.text)

    scores = EvidenceScores(
        vegetation_contact_probability=out.vegetation_contact_probability,
        impact_probability=out.impact_probability,
        report_credibility=out.report_credibility,
        innocent_explanation=out.innocent_explanation,
    )

    veg_p = out.vegetation_contact_probability
    imp_p = out.impact_probability
    priority = round(veg_p * imp_p, 4)

    evidence_count = sum(
        1 for p in [veg_p, imp_p, out.report_credibility] if p >= EVIDENCE_THRESHOLD
    )
    decision = _decide(priority, evidence_count)

    return EvaluationResult(
        segment_id=hypothesis.segment_id,
        vegetation_contact_probability=veg_p,
        impact_probability=imp_p,
        dispatch_priority=priority,
        decision=decision,
        evidence=scores,
        reasoning=_reasoning(
            hypothesis.segment_id, scores, veg_p, imp_p, priority, evidence_count, decision
        ),
    )
