"""Task telemetry model and readable summaries. No execution or robot control."""
from dataclasses import dataclass
import math

STAGES = (
    ('navigate', 'Navigate to table'),
    ('stabilize', 'Stop and stabilize'),
    ('observe', 'Observe target'),
    ('plan', 'Preview grasp plan'),
    ('grasp', 'Approach and close hand'),
    ('lift', 'Lift object'),
    ('confirm', 'Confirm result'),
)


@dataclass(frozen=True)
class TaskState:
    stage: str
    progress: float
    distance: float
    target_visible: bool = False
    grasp_confirmed: bool = False
    simulated: bool = False

    def __post_init__(self):
        if self.stage not in dict(STAGES):
            raise ValueError(f'Unknown task stage: {self.stage}')
        if not math.isfinite(self.progress) or not 0 <= self.progress <= 1:
            raise ValueError('Task progress must be in [0, 1]')
        if not math.isfinite(self.distance) or self.distance < 0:
            raise ValueError('Task distance must be finite and nonnegative')
        for name in ('target_visible', 'grasp_confirmed', 'simulated'):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f'{name} must be boolean')
        if self.grasp_confirmed and self.stage != 'confirm':
            raise ValueError('Grasp confirmation belongs to the result-confirmation stage')

    def markdown(self):
        index = [key for key, _ in STAGES].index(self.stage)
        complete = self.stage == 'confirm' and self.progress == 1 and self.grasp_confirmed
        fill = round(self.progress * 24)
        lines = ['# Navigation + grasp', '',
                 '**SYNTHETIC DEMO | no robot connection**' if self.simulated else '**Task telemetry | read-only**', '',
                 f'**{index + 1} / {len(STAGES)} — {STAGES[index][1]}** '
                 f'`[{"#" * fill}{"-" * (24 - fill)}]` **{self.progress:.0%}**', '',
                 '| Step | State |', '|---|---|']
        for i, (_, label) in enumerate(STAGES):
            status = 'DONE' if i < index or complete else ('>> ACTIVE' if i == index else 'WAITING')
            lines.append(f'| {i + 1}. {label} | {status} |')
        result = ('SIMULATED SUCCESS' if self.simulated else 'CONFIRMED') if self.grasp_confirmed else 'NOT CONFIRMED'
        lines += [f'\nDistance to stop pose: **{self.distance:.2f} m** · Target: **{"VISIBLE" if self.target_visible else "NOT OBSERVED"}**',
                  f'\nGrasp result: **{result}**']
        return '\n'.join(lines)
