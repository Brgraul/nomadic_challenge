"""
Crew-dispatch decision logic using Gemini structured outputs (Pydantic).

Gemini returns typed evidence scores; code maps them to a Decision:

  >= 0.8  AND  evidence_count >= 2  →  DISPATCH_CREW
  0.4 – 0.8                         →  MEASURE_MORE
  < 0.2                             →  DEPRIORITIZE
"""

import math
import os
from google import genai
from google.genai import types
from pydantic import BaseModel, Field

from .types import (
    Decision,
    EvidenceScores,
    EvaluationResult,
    SegmentHypothesis,
)
from .questions import build_prompt

DISPATCH_THRESHOLD = 0.80
DEPRIORITIZE_THRESHOLD = 0.20
EVIDENCE_THRESHOLD = 0.60
MIN_EVIDENCE_FOR_DISPATCH = 2


class EvidenceOutput(BaseModel):
    vegetation_risk: float = Field(
        description="0.0–1.0. Probability that a tree could physically contact this conductor given the vegetation context."
    )
    sensor_anomaly: float = Field(
        description="0.0–1.0. Probability that sensor readings indicate an anomaly inconsistent with normal post-storm conditions."
    )
    report_credibility: float = Field(
        description="0.0–1.0. Probability that incoming reports credibly and specifically implicate this segment as the fault location."
    )
    innocent_explanation: float = Field(
        description="0.0–1.0. Probability that a plausible benign explanation (planned switching, generator test, etc.) fully accounts for all observations."
    )


def _combine(veg: float, sensor: float, report: float, innocent: float) -> float:
    positive = [veg, sensor, report]
    geo = math.exp(sum(math.log(max(p, 1e-6)) for p in positive) / len(positive))
    discount = 1.0 - (innocent * 0.5)
    return round(geo * discount, 4)


def _decide(combined: float, evidence_count: int) -> Decision:
    if combined >= DISPATCH_THRESHOLD and evidence_count >= MIN_EVIDENCE_FOR_DISPATCH:
        return Decision.DISPATCH_CREW
    if combined < DEPRIORITIZE_THRESHOLD:
        return Decision.DEPRIORITIZE
    return Decision.MEASURE_MORE


def _reasoning(seg, scores, combined, evidence_count, decision) -> str:
    lines = [
        f"Segment {seg}: combined P={combined:.2f}, evidence_count={evidence_count}",
        f"  vegetation_risk={scores.vegetation_risk:.2f}  "
        f"sensor_anomaly={scores.sensor_anomaly:.2f}  "
        f"report_credibility={scores.report_credibility:.2f}  "
        f"innocent_explanation={scores.innocent_explanation:.2f}",
        f"  → {decision.value}",
    ]
    if decision == Decision.DISPATCH_CREW:
        lines.append("  Action: dispatch an operator crew for physical inspection.")
    elif decision == Decision.MEASURE_MORE:
        lines.append("  Action: send a drone for additional sensor readings before committing a crew.")
    else:
        lines.append("  Action: deprioritize; reallocate drone capacity to higher-confidence segments.")
    return "\n".join(lines)


def evaluate(hypothesis: SegmentHypothesis) -> EvaluationResult:
    """Evaluate a single segment hypothesis using Gemini structured output."""
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
        vegetation_risk=out.vegetation_risk,
        sensor_anomaly=out.sensor_anomaly,
        report_credibility=out.report_credibility,
        innocent_explanation=out.innocent_explanation,
    )

    evidence_count = sum(
        1 for p in [out.vegetation_risk, out.sensor_anomaly, out.report_credibility]
        if p >= EVIDENCE_THRESHOLD
    )
    combined = _combine(out.vegetation_risk, out.sensor_anomaly, out.report_credibility, out.innocent_explanation)
    decision = _decide(combined, evidence_count)

    return EvaluationResult(
        segment_id=hypothesis.segment_id,
        combined_probability=combined,
        evidence_count=evidence_count,
        decision=decision,
        evidence=scores,
        reasoning=_reasoning(hypothesis.segment_id, scores, combined, evidence_count, decision),
    )
