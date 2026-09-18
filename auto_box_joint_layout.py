"""Deterministic sizing and safety checks for the Auto Box Joint command.

This module deliberately has no Autodesk Fusion dependency so the print-fit
rules can be verified with ordinary Python unit tests.
"""

from dataclasses import dataclass
from typing import Optional, Tuple


def toolbar_placements() -> tuple[tuple[str, str, bool], ...]:
    """Expose one normal menu entry that the user may pin manually."""
    return (('FusionSolidEnvironment', 'SolidModifyPanel', False),)


def command_resource_parts() -> tuple[str, str]:
    """Return the relative directory containing Fusion command icons."""
    return ('Resources', 'BoxJoint')


def selection_is_allowed(active_input_id: str, is_preview_face: bool) -> bool:
    """Only the real overlap preview can supply the mating face."""
    if active_input_id == 'matingFace':
        return is_preview_face
    return True


def should_create_overlap_preview(selected_body_count: int, preview_ready: bool,
                                  preview_attempted: bool = False) -> bool:
    """Create one overlap preview as soon as exactly two bodies are selected."""
    return selected_body_count == 2 and not preview_ready and not preview_attempted


def restored_source_selectability(_observed_during_selection: bool) -> bool:
    """A body accepted by the source picker was selectable before selection."""
    return True


def should_reset_body_selection_cache(selection_count: int,
                                      preview_ready: bool,
                                      preview_creating: bool) -> bool:
    """Ignore transient selection loss caused while constructing the preview."""
    return selection_count < 2 and not preview_ready and not preview_creating


def preview_storage_strategy(is_direct_design: bool) -> str:
    """Choose the Fusion persistence mechanism supported by the design type."""
    return 'direct_body' if is_direct_design else 'base_feature'


def is_legacy_stale_source_state(opacity: float, is_selectable: bool) -> bool:
    """Match the exact state left by the pre-recovery preview implementation."""
    return abs(opacity - 0.18) <= 1e-6 and not is_selectable


def clip_candidate_to_overlap(candidate, overlap_body, copy_body, intersect_body):
    """Return the candidate clipped by the true overlap solid, or None."""
    clipped = copy_body(overlap_body)
    if not intersect_body(clipped, candidate):
        return None
    if not clipped.isSolid or clipped.volume <= 0:
        return None
    return clipped


@dataclass(frozen=True)
class JointLimits:
    min_finger_mm: float = 3.0
    min_margin_mm: float = 1.5
    min_wall_mm: float = 2.0


@dataclass(frozen=True)
class LayoutRequest:
    axis_lengths_mm: Tuple[float, float, float]
    selected_axis: Optional[int]
    tooth_count: int
    lateral_clearance_mm: float
    axial_clearance_mm: float


@dataclass(frozen=True)
class JointLayout:
    valid: bool
    message: str
    axis_index: int
    usable_length_mm: float
    finger_width_mm: float
    end_margin_mm: float
    max_odd_tooth_count: int
    tooth_count: int = 0


@dataclass(frozen=True)
class ShortTenonRequest:
    overlap_length_mm: float
    insertion_axis: int
    body_a_negative_extension_mm: float
    body_a_positive_extension_mm: float
    body_b_negative_extension_mm: float
    body_b_positive_extension_mm: float


@dataclass(frozen=True)
class ShortTenonPlan:
    valid: bool
    message: str
    insertion_axis: int
    tenon_owner: str
    receiver: str
    entry_side: str
    tenon_depth_mm: float


@dataclass(frozen=True)
class FaceJointRequest:
    overlap_lengths_mm: Tuple[float, float, float]
    face_normal_axis: int
    body_a_extensions_mm: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]
    body_b_extensions_mm: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]
    tooth_count: int
    clearance_mm: float


@dataclass(frozen=True)
class FaceJointPlan:
    valid: bool
    message: str
    distribution_axis: int
    depth_axis: int
    receiver: str
    tenon_owner: str
    entry_side: str
    tooth_width_mm: float
    tooth_depth_mm: float
    tooth_count: int
    clearance_mm: float


def _failed(message: str, axis_index: int, usable_length_mm: float,
            max_odd_tooth_count: int, end_margin_mm: float) -> JointLayout:
    return JointLayout(
        valid=False,
        message=message,
        axis_index=axis_index,
        usable_length_mm=usable_length_mm,
        finger_width_mm=0.0,
        end_margin_mm=end_margin_mm,
        max_odd_tooth_count=max_odd_tooth_count,
    )


def plan_layout(request: LayoutRequest,
                limits: JointLimits = JointLimits()) -> JointLayout:
    """Return an equal-width, odd-tooth layout or a Chinese validation error."""
    if len(request.axis_lengths_mm) != 3:
        raise ValueError('axis_lengths_mm 必须包含三个方向长度。')

    if request.selected_axis is None:
        longest = max(request.axis_lengths_mm)
        axis_index = next(
            index for index, length in enumerate(request.axis_lengths_mm)
            if longest - length <= 0.01)
    else:
        axis_index = request.selected_axis
    if axis_index not in (0, 1, 2):
        raise ValueError('selected_axis 必须为 0、1、2 或 None。')

    lengths = request.axis_lengths_mm
    end_margin_mm = limits.min_margin_mm
    usable_length_mm = lengths[axis_index] - 2.0 * end_margin_mm
    cross_section_mm = min(lengths[index] for index in range(3) if index != axis_index)
    max_odd_tooth_count = int(usable_length_mm // limits.min_finger_mm)
    if max_odd_tooth_count % 2 == 0:
        max_odd_tooth_count -= 1

    if cross_section_mm < 2.0 * limits.min_wall_mm:
        return _failed(
            f'所选方向的截面厚度不足 {2.0 * limits.min_wall_mm:.1f} mm，无法保留两侧壁厚。',
            axis_index,
            usable_length_mm,
            max(0, max_odd_tooth_count),
            end_margin_mm,
        )

    if request.tooth_count < 3 or request.tooth_count % 2 == 0:
        return _failed(
            '榫齿数必须为不小于 3 的奇数。',
            axis_index,
            usable_length_mm,
            max(0, max_odd_tooth_count),
            end_margin_mm,
        )

    if max_odd_tooth_count < 3:
        return _failed(
            '所选方向长度不足，无法容纳至少 3 个榫齿。',
            axis_index,
            usable_length_mm,
            max(0, max_odd_tooth_count),
            end_margin_mm,
        )

    if request.tooth_count > max_odd_tooth_count:
        return _failed(
            f'所选方向最多可用 {max_odd_tooth_count} 齿；请减少榫齿数。',
            axis_index,
            usable_length_mm,
            max_odd_tooth_count,
            end_margin_mm,
        )

    finger_width_mm = usable_length_mm / request.tooth_count
    return JointLayout(
        valid=True,
        message=(
            f'方向：{("X", "Y", "Z")[axis_index]}；'
            f'{request.tooth_count} 齿，每齿 {finger_width_mm:.2f} mm；'
            f'端部余量 {end_margin_mm:.2f} mm。'
        ),
        axis_index=axis_index,
        usable_length_mm=usable_length_mm,
        finger_width_mm=finger_width_mm,
        end_margin_mm=end_margin_mm,
        max_odd_tooth_count=max_odd_tooth_count,
        tooth_count=request.tooth_count,
    )


def plan_short_tenon(request: ShortTenonRequest) -> ShortTenonPlan:
    """Plan a straight, short mortise-and-tenon from one directed overlap."""
    if request.insertion_axis not in (0, 1, 2):
        raise ValueError('insertion_axis 必须为 0、1 或 2。')
    if request.overlap_length_mm < 10.0:
        return ShortTenonPlan(
            False, '相交长度不足 10.0 mm，无法建立短榫。', request.insertion_axis,
            '', '', '', 0.0)

    extensions = {
        ('a', 'min'): request.body_a_negative_extension_mm,
        ('a', 'max'): request.body_a_positive_extension_mm,
        ('b', 'min'): request.body_b_negative_extension_mm,
        ('b', 'max'): request.body_b_positive_extension_mm,
    }
    receiver, receiver_side = max(extensions, key=extensions.__getitem__)
    if extensions[(receiver, receiver_side)] <= 0.01:
        return ShortTenonPlan(
            False, '无法由实体延伸判断榫头与榫槽；请调整实体或指定装配方向。',
            request.insertion_axis, '', '', '', 0.0)
    tenon_owner = 'b' if receiver == 'a' else 'a'
    entry_side = 'min' if receiver_side == 'max' else 'max'
    tenon_depth_mm = min(20.0, request.overlap_length_mm * 0.4)
    axis_name = ('X', 'Y', 'Z')[request.insertion_axis]
    return ShortTenonPlan(
        True,
        f'短榫沿 {axis_name} 插入，深度 {tenon_depth_mm:.2f} mm。',
        request.insertion_axis,
        tenon_owner,
        receiver,
        entry_side,
        tenon_depth_mm,
    )


def plan_face_joint(request: FaceJointRequest) -> FaceJointPlan:
    """Plan a box joint from the normal of its user-selected mating face."""
    if len(request.overlap_lengths_mm) != 3:
        raise ValueError('overlap_lengths_mm 必须包含三个方向长度。')
    if request.face_normal_axis not in (0, 1, 2):
        raise ValueError('配合面法线必须为 X、Y 或 Z 方向。')
    if request.tooth_count < 2:
        return FaceJointPlan(False, '榫片数量至少为 2。', request.face_normal_axis, -1,
                             '', '', '', 0.0, 0.0, request.tooth_count, request.clearance_mm)
    if request.clearance_mm < 0:
        return FaceJointPlan(False, '配合间隙不能小于 0。', request.face_normal_axis, -1,
                             '', '', '', 0.0, 0.0, request.tooth_count, request.clearance_mm)

    distribution_axis = request.face_normal_axis
    distribution_length = request.overlap_lengths_mm[distribution_axis]
    if distribution_length <= 0:
        return FaceJointPlan(False, '配合面法线方向的相交长度必须大于 0。', distribution_axis, -1,
                             '', '', '', 0.0, 0.0, request.tooth_count, request.clearance_mm)
    tooth_width = distribution_length / request.tooth_count
    if tooth_width < 3.0:
        return FaceJointPlan(False, '每片榫片宽度不足 3.0 mm；请减少榫片数量。', distribution_axis, -1,
                             '', '', '', tooth_width, 0.0, request.tooth_count, request.clearance_mm)

    axes = ('X', 'Y', 'Z')
    return FaceJointPlan(
        True,
        f'配合面法线为 {axes[distribution_axis]}；整个相交区沿 '
        f'{axes[distribution_axis]} 分为 {request.tooth_count} 片并交替归属。',
        distribution_axis, -1, '', '', '', tooth_width, 0.0, request.tooth_count,
        request.clearance_mm)
