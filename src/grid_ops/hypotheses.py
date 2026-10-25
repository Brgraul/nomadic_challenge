"""Hypotheses, explained revisions and the manager's structured output.

The manager owns hypotheses, urgency and recommendations. A hypothesis states a falsifiable
claim with a spatial scope, predictions and required conditions. Its `assessment` is qualitative
evidence support; `priority` (urgency 1 to 5) is a separate, manager-owned operational judgement
and is never a probability. Every change carries before/after values, a finding, a causal update
reason and the remaining uncertainty.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Literal

from pydantic import BaseModel, Field, field_validator

from grid_ops.sensing import MeasurementRequest


class Assessment(StrEnum):
    SUPPORTED = "SUPPORTED"
    PLAUSIBLE = "PLAUSIBLE"
    UNRESOLVED = "UNRESOLVED"
    UNLIKELY = "UNLIKELY"
    REFUTED = "REFUTED"


class Lifecycle(StrEnum):
    ACTIVE = "ACTIVE"
    RETIRED = "RETIRED"


ChangeKind = Literal["creation", "assessment", "scope", "prediction", "priority", "lifecycle"]
TriggerKind = Literal["startup", "measurement", "operator", "none"]
ActionKind = Literal["inspect", "repair", "measure", "monitor", "none"]
MeasurementName = Literal["voltage_presence", "current", "conductor_temp", "clearance", "motion"]


class SpanModel(BaseModel):
    segment_id: str
    start_km: float = Field(ge=0)
    end_km: float = Field(ge=0)

    @field_validator("end_km")
    @classmethod
    def _ordered(cls, v: float, info) -> float:
        if "start_km" in info.data and v < info.data["start_km"]:
            raise ValueError("end_km must not precede start_km")
        return v


class PredictionModel(BaseModel):
    id: str
    statement: str
    measurement: MeasurementName
    expected: str = Field(description="what the measurement should show if the claim is true")
    required_conditions: list[str] = Field(
        default_factory=list, description="e.g. 'thermal excitation above 20 A', 'qualifying gust'"
    )


class HypothesisModel(BaseModel):
    id: str
    claim: str = Field(description="falsifiable claim")
    scope: list[SpanModel]
    predictions: list[PredictionModel]
    assessment: Assessment
    priority: int = Field(ge=1, le=5, description="manager-owned urgency, not a probability")
    lifecycle: Lifecycle = Lifecycle.ACTIVE
    supporting_evidence_ids: list[str] = Field(default_factory=list)
    contradicting_evidence_ids: list[str] = Field(default_factory=list)
    model_change: list[str] | None = Field(
        default=None,
        description="candidate topology: segments assumed out of service (staged, tested first)",
    )


class CitedValue(BaseModel):
    reading_id: str
    quantity: str = Field(description="payload name, e.g. 'clearance_m'")
    value: float
    unit: str
    location: str = Field(description="segment and offset, e.g. 'LINE_058 @ 0.029 km'")
    conditions: str = Field(description="detection conditions as recorded on the reading")


class RevisionModel(BaseModel):
    hypothesis_id: str
    change_kinds: list[ChangeKind]
    before: HypothesisModel | None
    after: HypothesisModel
    finding_summary: str
    update_reason: str
    remaining_uncertainty: str
    trigger_kind: TriggerKind
    trigger_event_ids: list[str] = Field(default_factory=list)
    operator_event_id: str | None = None
    contributing_evidence_ids: list[str] = Field(default_factory=list)
    latest_measurement_ids: list[str] = Field(default_factory=list)
    contrary_evidence_ids: list[str] = Field(default_factory=list)
    diagnostic_ids: list[str] = Field(default_factory=list)
    cited_values: list[CitedValue] = Field(default_factory=list)


class RecommendationModel(BaseModel):
    id: str
    action_kind: ActionKind
    text: str
    scope: SpanModel | None = None
    urgency: int = Field(ge=1, le=5)
    evidence_ids: list[str]
    rationale: str
    uncertainty: str


class TurnAssessment(BaseModel):
    hypothesis_id: str
    assessment_text: str = Field(description="why this hypothesis is unchanged this turn")


class ManagerOutput(BaseModel):
    summary: str
    revisions: list[RevisionModel] = Field(default_factory=list)
    unchanged: list[TurnAssessment] = Field(default_factory=list)
    recommendations: list[RecommendationModel] = Field(default_factory=list)
    untested_alternatives: list[str] = Field(
        default_factory=list,
        description="candidates that could not be distinguished or tested with this evidence",
    )
    conclusion: ConclusionModel | None = None
    reopen: ReopenModel | None = None
    measurement_requests: list[MeasurementRequest] = Field(default_factory=list)
    retire_request_ids: list[str] = Field(default_factory=list)


class AlternativeDisposition(BaseModel):
    hypothesis_id: str
    disposition: Literal["refuted", "unlikely", "indistinguishable", "not_material"]
    evidence_ids: list[str] = Field(default_factory=list)
    note: str = ""


class ConclusionModel(BaseModel):
    """A proposal to stop diagnostic collection (or to say the cause is still ambiguous)."""

    kind: Literal["cause_identified", "unresolved_ambiguous"]
    cause_hypothesis_id: str | None = None
    causal_statement: str
    causal_link_evidence_ids: list[str] = Field(
        default_factory=list,
        description="evidence linking the cause to the OUTAGE (not just a local hazard)",
    )
    alternatives: list[AlternativeDisposition] = Field(default_factory=list)
    recommendation_id: str | None = None
    discriminating_recommendation_id: str | None = None
    remaining_uncertainty: str


class ReopenModel(BaseModel):
    reason: str
    evidence_ids: list[str]


ManagerOutput.model_rebuild()
