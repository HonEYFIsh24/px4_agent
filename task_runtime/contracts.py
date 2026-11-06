"""Versioned Agent-side contracts. Executor messages remain plain JSON."""
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Literal, Optional

from pydantic import BaseModel, ConfigDict, Field


class TaskProtocolError(ValueError):
    """Unexecuted model/protocol violation; never an operator or flight fault."""


class StrictInput(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)


class TargetProposal(StrictInput):
    ref: str = Field(min_length=1, max_length=64, pattern=r'^[a-zA-Z][a-zA-Z0-9_-]*$')
    label: str = Field(min_length=1, max_length=80)
    attributes: Dict[str, str] = Field(default_factory=dict)


class StageProposal(StrictInput):
    kind: Literal['search', 'close_photo', 'approach', 'photo', 'report', 'relative_hover', 'relative_land',
                  'pass_landmark', 'face_target', 'relative_turn',
                  'return_home_land', 'hold']
    target_ref: Optional[str] = None
    relation: Optional[Literal['object_front', 'object_behind', 'object_near', 'object_beside']] = None
    altitude_m: Optional[float] = Field(default=None, ge=1., le=20.)
    direction: Optional[Literal['left', 'right']] = None
    angle_deg: Optional[float] = Field(default=None, ge=5., le=90.)


class PlanProposal(StrictInput):
    targets: List[TargetProposal] = Field(min_length=1, max_length=8)
    stages: List[StageProposal] = Field(min_length=1, max_length=24)


@dataclass(frozen=True)
class TaskLimits:
    max_observations: int
    max_duration_seconds: float
    max_model_calls: int
    max_recoveries: int
    radius_m: float
    max_altitude_m: float


@dataclass
class TaskStage:
    stage_id: str
    kind: str
    target_ref: Optional[str]
    relation: Optional[str]
    evidence_profile: str
    status: str = 'pending'
    conditions: dict = field(default_factory=dict)  # true/false/unknown
    evidence_refs: list = field(default_factory=list)
    reason_code: Optional[str] = None
    altitude_m: Optional[float] = None
    direction: Optional[str] = None
    angle_deg: Optional[float] = None


@dataclass
class TaskPlan:
    original_instruction: str
    owner: tuple
    limits: TaskLimits
    targets: list
    stages: list
    plan_revision: int = 1
    schema_version: int = 1
    failure_policy: str = 'return_home_if_healthy_else_owned_safety_cleanup'

    def snapshot(self):
        return asdict(self)


@dataclass
class SearchOutcome:
    target_id: str
    capture_id: str
    query: str
    confirmation_level: str
    coordinate_epoch: dict
    stage_id: str
    schema_version: int = 1
    status: str = 'evidence_ready'


@dataclass
class ObservationNeed:
    kind: str
    target_id: str
    source_capture_id: str
    missing_information: list
    acceptable_evidence: list
    schema_version: int = 1


@dataclass
class TargetState:
    target_id: str
    label: str
    observation_refs: list
    position_enu_m: Optional[list]
    covariance_enu_m2: Optional[list]
    confirmation_level: str
    coordinate_epoch: dict
    world_sha256: str
    sortie_id: str
    orientation: Optional[dict] = None
    extent_known: bool = False
    schema_version: int = 1


@dataclass(frozen=True)
class GoalSpec:
    target_id: str
    relation: str
    terminal_action: str
    facing: str = 'target'
    reference_frame: str = 'object'
    schema_version: int = 1


@dataclass
class GoalPoseCandidate:
    id: str
    position_enu_m: list
    yaw_enu_rad: float
    goal_spec: dict
    evidence_refs: list
    target_revision: int
    map_revision: str
    coordinate_epoch: dict
    hard_constraints: dict
    score_terms: dict = field(default_factory=dict)
    score: Optional[float] = None
    path: Optional[dict] = None
    schema_version: int = 1


@dataclass
class SkillResult:
    status: str  # completed / needs_observation / unavailable / cancelled / safety_fault
    reason_code: Optional[str] = None
    evidence_refs: list = field(default_factory=list)
    actual_mission_id: Optional[str] = None
    schema_version: int = 1


@dataclass
class TaskResult:
    process_status: str
    goal_status: str
    completed_stages: list
    unmet_requirements: list
    evidence_summary: dict
    flight_status: str
    termination_source: str
    schema_version: int = 1

    def snapshot(self):
        return asdict(self)
