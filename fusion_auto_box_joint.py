"""Fusion command shell for the Chinese Auto Box Joint workflow."""

from dataclasses import dataclass
import json
import os

import adsk.core
import adsk.fusion

from .auto_box_joint_layout import (
    FaceJointPlan, FaceJointRequest, JointLayout, ShortTenonPlan, ShortTenonRequest,
    clip_candidate_to_overlap, plan_face_joint, plan_short_tenon,
    command_resource_parts,
    is_legacy_stale_source_state, preview_storage_strategy,
    restored_source_selectability, selection_is_allowed,
    should_create_overlap_preview, should_reset_body_selection_cache,
    toolbar_placements,
)
from .fusion_brep_util import createSimpleBox, intersect, tempBrepMgr
from .fusion_util import handleException, newEventHandler


COMMAND_ID = 'Codex_AutoBoxJointCreate'
RESTORE_EVENT_ID = 'Codex_AutoBoxJointRestoreSources'


@dataclass
class IntersectionInfo:
    body_a: adsk.fusion.BRepBody
    body_b: adsk.fusion.BRepBody
    body: adsk.fusion.BRepBody
    axis_lengths_mm: tuple[float, float, float]
    center: adsk.core.Point3D


@dataclass
class SegmentTool:
    target_body: adsk.fusion.BRepBody
    body: adsk.fusion.BRepBody
    target_is_first_body: bool


class OverlapPreviewSession:
    """Owns the one selectable, short-lived overlap body for a command session."""

    BODY_NAME = '自动箱形榫-相交预览'
    STATE_GROUP = 'CodexAutoBoxJoint'
    STATE_NAME = 'previewSourceState'

    def __init__(self):
        self._base_feature = None
        self._body = None
        self._body_token = None
        self._source_states = []

    @property
    def body(self):
        return self._body

    def create(self, info: IntersectionInfo) -> adsk.fusion.BRepBody:
        self.clear()
        design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
        component = design.activeComponent
        is_direct = design.designType == adsk.fusion.DesignTypes.DirectDesignType
        feature = None
        if preview_storage_strategy(is_direct) == 'direct_body':
            body = component.bRepBodies.add(tempBrepMgr.copy(info.body))
        else:
            feature = component.features.baseFeatures.add()
            feature.name = self.BODY_NAME
            feature.startEdit()
            component.bRepBodies.add(tempBrepMgr.copy(info.body), feature)
            feature.finishEdit()
            body = feature.bodies.item(0)
        if not body:
            raise ValueError('无法在当前设计模式中建立相交预览体。')
        body.name = self.BODY_NAME
        self._base_feature = feature
        self._body = body
        self._body_token = body.entityToken
        for source in (info.body_a, info.body_b):
            opacity = source.opacity
            is_selectable = restored_source_selectability(source.isSelectable)
            self._source_states.append((source, opacity, is_selectable))
            source.attributes.add(
                self.STATE_GROUP,
                self.STATE_NAME,
                json.dumps({'opacity': opacity, 'isSelectable': is_selectable}),
            )
            source.opacity = 0.18
            source.isSelectable = False
        body.opacity = 0.80
        return body

    def is_preview_face(self, entity) -> bool:
        face = adsk.fusion.BRepFace.cast(entity)
        return bool(
            face and self._body_token and face.body and
            face.body.entityToken == self._body_token)

    def clear(self) -> None:
        sources = self._source_states
        self._source_states = []
        for source, opacity, is_selectable in sources:
            try:
                if source and source.isValid:
                    source.opacity = opacity
                    source.isSelectable = is_selectable
                    attribute = source.attributes.itemByName(
                        self.STATE_GROUP, self.STATE_NAME)
                    if attribute:
                        attribute.deleteMe()
            except Exception:
                handleException()

        feature = self._base_feature
        body = self._body
        self._base_feature = None
        self._body = None
        self._body_token = None
        deleted = False
        if feature and feature.isValid:
            try:
                feature.deleteMe()
                deleted = True
            except Exception:
                handleException()
        if not deleted and body and body.isValid:
            try:
                body.deleteMe()
            except Exception:
                handleException()

    @classmethod
    def recover_orphaned_state(cls, design) -> int:
        """Restore sources left dimmed by an interrupted command session."""
        restored = 0
        legacy_candidates = []
        preview_bodies = []
        for component in design.allComponents:
            for body in component.bRepBodies:
                if body.name == cls.BODY_NAME:
                    preview_bodies.append(body)
                    continue
                attribute = body.attributes.itemByName(cls.STATE_GROUP, cls.STATE_NAME)
                if attribute:
                    try:
                        state = json.loads(attribute.value)
                        body.opacity = float(state['opacity'])
                        # A marked body was accepted by the source picker, so
                        # any observed false value was Fusion's transient
                        # selection state rather than its original state.
                        body.isSelectable = True
                        attribute.deleteMe()
                        restored += 1
                    except Exception:
                        handleException()
                elif is_legacy_stale_source_state(body.opacity, body.isSelectable):
                    legacy_candidates.append(body)

        # The previous implementation did not persist recovery attributes.
        # Recover only the exact two-body footprint confirmed by the user.
        if len(legacy_candidates) == 2:
            for body in legacy_candidates:
                body.opacity = 1.0
                body.isSelectable = True
                restored += 1

        for body in preview_bodies:
            try:
                if body.isValid:
                    body.deleteMe()
            except Exception:
                handleException()
        return restored


def _body_axis_extensions(body: adsk.fusion.BRepBody, overlap_box, axis: int) -> tuple[float, float]:
    """Return the body length that continues beyond the overlap on both ends."""
    body_min = (body.boundingBox.minPoint.x, body.boundingBox.minPoint.y, body.boundingBox.minPoint.z)[axis]
    body_max = (body.boundingBox.maxPoint.x, body.boundingBox.maxPoint.y, body.boundingBox.maxPoint.z)[axis]
    overlap_min = (overlap_box.minPoint.x, overlap_box.minPoint.y, overlap_box.minPoint.z)[axis]
    overlap_max = (overlap_box.maxPoint.x, overlap_box.maxPoint.y, overlap_box.maxPoint.z)[axis]
    return max(0.0, (overlap_min - body_min) * 10.0), max(0.0, (body_max - overlap_max) * 10.0)


def plan_short_tenon_from_bodies(info: IntersectionInfo, insertion_axis: int) -> ShortTenonPlan:
    """Choose the receiver from the body that continues past the overlap."""
    overlap_box = info.body.boundingBox
    a_negative, a_positive = _body_axis_extensions(info.body_a, overlap_box, insertion_axis)
    b_negative, b_positive = _body_axis_extensions(info.body_b, overlap_box, insertion_axis)
    return plan_short_tenon(ShortTenonRequest(
        overlap_length_mm=info.axis_lengths_mm[insertion_axis],
        insertion_axis=insertion_axis,
        body_a_negative_extension_mm=a_negative,
        body_a_positive_extension_mm=a_positive,
        body_b_negative_extension_mm=b_negative,
        body_b_positive_extension_mm=b_positive,
    ))


def build_short_tenon_tools(info: IntersectionInfo, plan: ShortTenonPlan) -> list[SegmentTool]:
    """Build two end-based cutters for one straight, short rectangular tenon.

    The receiver is opened only from its insertion end.  The tenon owner's
    remaining overlap is removed behind that end region, so the bodies do not
    overlap while each stays structurally connected.
    """
    if not plan.valid:
        raise ValueError(plan.message)
    box = info.body.boundingBox
    lower = [box.minPoint.x, box.minPoint.y, box.minPoint.z]
    upper = [box.maxPoint.x, box.maxPoint.y, box.maxPoint.z]
    axis = plan.insertion_axis
    overlap_length = upper[axis] - lower[axis]
    depth = plan.tenon_depth_mm / 10.0
    if depth <= 0 or depth >= overlap_length:
        raise ValueError('短榫深度必须小于相交长度。')

    tenon_lower = lower.copy()
    tenon_upper = upper.copy()
    relief_lower = lower.copy()
    relief_upper = upper.copy()
    if plan.entry_side == 'min':
        tenon_upper[axis] = lower[axis] + depth
        relief_lower[axis] = lower[axis] + depth
    else:
        tenon_lower[axis] = upper[axis] - depth
        relief_upper[axis] = upper[axis] - depth

    receiver = info.body_a if plan.receiver == 'a' else info.body_b
    tenon_owner = info.body_a if plan.tenon_owner == 'a' else info.body_b
    mortise = createSimpleBox(
        tenon_lower[0], tenon_lower[1], tenon_lower[2],
        tenon_upper[0] - tenon_lower[0],
        tenon_upper[1] - tenon_lower[1],
        tenon_upper[2] - tenon_lower[2])
    relief = createSimpleBox(
        relief_lower[0], relief_lower[1], relief_lower[2],
        relief_upper[0] - relief_lower[0],
        relief_upper[1] - relief_lower[1],
        relief_upper[2] - relief_lower[2])
    result = []
    for target, candidate in ((receiver, mortise), (tenon_owner, relief)):
        clipped = tempBrepMgr.copy(candidate)
        if intersect(clipped, target) and clipped.isSolid and clipped.volume > 0:
            result.append(SegmentTool(target, clipped, target == info.body_a))
    if len(result) != 2:
        raise ValueError('短榫工具体未能完整落在两个目标实体内。')
    return result


def calculate_intersection(body_a: adsk.fusion.BRepBody,
                           body_b: adsk.fusion.BRepBody) -> IntersectionInfo:
    """Return one non-empty temporary intersection body in root coordinates."""
    overlap = tempBrepMgr.copy(body_a)
    if not intersect(overlap, body_b) or not overlap.isSolid or overlap.volume <= 0:
        raise ValueError('两个实体没有体积相交；请移动或重新选择实体。')
    if overlap.lumps.count != 1:
        raise ValueError('相交体包含多个独立区域；请先拆分实体或选择目标区域。')

    box = overlap.boundingBox
    lengths_cm = (
        box.maxPoint.x - box.minPoint.x,
        box.maxPoint.y - box.minPoint.y,
        box.maxPoint.z - box.minPoint.z,
    )
    return IntersectionInfo(
        body_a=body_a,
        body_b=body_b,
        body=overlap,
        axis_lengths_mm=tuple(value * 10.0 for value in lengths_cm),
        center=adsk.core.Point3D.create(
            (box.minPoint.x + box.maxPoint.x) / 2.0,
            (box.minPoint.y + box.maxPoint.y) / 2.0,
            (box.minPoint.z + box.maxPoint.z) / 2.0,
        ),
    )


def build_segment_tools(info: IntersectionInfo, layout: JointLayout,
                        lateral_clearance_mm: float,
                        axial_clearance_mm: float,
                        first_body_owns_first: bool) -> list[SegmentTool]:
    """Build alternating clipped temporary cutters from the true overlap box.

    Each alternating slice is removed from one body, leaving the other body as
    the owner of that slice. The side clearance enlarges each cut along the
    tooth-distribution axis; axial clearance enlarges the cut through its two
    perpendicular axes.
    """
    box = info.body.boundingBox
    mins = [box.minPoint.x, box.minPoint.y, box.minPoint.z]
    maxs = [box.maxPoint.x, box.maxPoint.y, box.maxPoint.z]
    axis = layout.axis_index
    cross_axes = [index for index in range(3) if index != axis]
    margin_cm = layout.end_margin_mm / 10.0
    lateral_cm = max(lateral_clearance_mm, 0.0) / 10.0
    axial_cm = max(axial_clearance_mm, 0.0) / 10.0
    finger_cm = layout.finger_width_mm / 10.0
    tools = []

    for index in range(round(layout.usable_length_mm / layout.finger_width_mm)):
        lower = mins.copy()
        upper = maxs.copy()
        start = mins[axis] + margin_cm + index * finger_cm - lateral_cm / 2.0
        end = mins[axis] + margin_cm + (index + 1) * finger_cm + lateral_cm / 2.0
        lower[axis] = max(mins[axis] + margin_cm, start)
        upper[axis] = min(maxs[axis] - margin_cm, end)
        for cross_axis in cross_axes:
            lower[cross_axis] -= axial_cm / 2.0
            upper[cross_axis] += axial_cm / 2.0
        slab = createSimpleBox(
            lower[0], lower[1], lower[2],
            upper[0] - lower[0], upper[1] - lower[1], upper[2] - lower[2])
        first_owns_slice = (index % 2 == 0) == first_body_owns_first
        target = info.body_b if first_owns_slice else info.body_a
        clipped = tempBrepMgr.copy(slab)
        if intersect(clipped, target) and clipped.isSolid and clipped.volume > 0:
            tools.append(SegmentTool(target, clipped, target == info.body_a))
    return tools


def build_face_joint_tools(info: IntersectionInfo, plan: FaceJointPlan,
                           first_body_owns_first: bool = True) -> list[SegmentTool]:
    """Build complementary full-overlap box-joint cutters from one mating face."""
    if not plan.valid:
        raise ValueError(plan.message)
    box = info.body.boundingBox
    mins = [box.minPoint.x, box.minPoint.y, box.minPoint.z]
    maxs = [box.maxPoint.x, box.maxPoint.y, box.maxPoint.z]
    axis = plan.distribution_axis
    band_width = plan.tooth_width_mm / 10.0
    clearance = plan.clearance_mm / 10.0
    tools = []
    for index in range(plan.tooth_count):
        lower = mins.copy()
        upper = maxs.copy()
        lower[axis] = max(mins[axis], mins[axis] + index * band_width - clearance / 2.0)
        upper[axis] = min(maxs[axis], mins[axis] + (index + 1) * band_width + clearance / 2.0)
        cutter = createSimpleBox(
            lower[0], lower[1], lower[2],
            upper[0] - lower[0], upper[1] - lower[1], upper[2] - lower[2])
        first_owns_band = (index % 2 == 0) == first_body_owns_first
        target = info.body_b if first_owns_band else info.body_a
        clipped = clip_candidate_to_overlap(
            cutter, info.body, tempBrepMgr.copy, intersect)
        if not clipped:
            raise ValueError(f'第 {index + 1} 片榫齿未能落在真实相交体内。')
        tools.append(SegmentTool(target, clipped, target == info.body_a))
    return tools


def apply_standard_cuts(tools: list[SegmentTool], layout: JointLayout | None = None,
                        label: str = '自动箱形榫') -> list[adsk.fusion.Feature]:
    """Create regular Fusion Cut features without cross-body join/intersect steps."""
    if not tools:
        raise ValueError('没有可切除的榫齿区域；未创建任何特征。')
    design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    component = design.activeComponent
    is_direct = design.designType == adsk.fusion.DesignTypes.DirectDesignType
    before_volumes = {}
    expected_removed = {}
    for tool in tools:
        if not tool.body.isSolid or tool.body.volume <= 0.0001:
            continue
        target = tool.target_body
        if target.parentComponent != component:
            raise ValueError('目标实体不在当前编辑组件中；请在根组件编辑模式下执行。')
        token = target.entityToken
        before_volumes[token] = target.volume
        expected_removed[token] = expected_removed.get(token, 0.0) + tool.body.volume
    if not before_volumes:
        raise ValueError('所有榫齿切除体均为空；未创建任何特征。')

    created = []
    for tool_index, tool in enumerate(tools, 1):
        if not tool.body.isSolid or tool.body.volume <= 0.0001:
            continue
        token = tool.target_body.entityToken
        target = adsk.fusion.BRepBody.cast(design.findEntityByToken(token)[0])
        base_feature = None
        if preview_storage_strategy(is_direct) == 'direct_body':
            fusion_tool_body = component.bRepBodies.add(tempBrepMgr.copy(tool.body))
        else:
            base_feature = component.features.baseFeatures.add()
            base_feature.name = f'{label}-工具-{tool_index:02d}'
            base_feature.startEdit()
            component.bRepBodies.add(tempBrepMgr.copy(tool.body), base_feature)
            base_feature.finishEdit()
            fusion_tool_body = base_feature.bodies.item(0)
        fusion_tools = adsk.core.ObjectCollection.create()
        fusion_tools.add(fusion_tool_body)
        combine_input = component.features.combineFeatures.createInput(target, fusion_tools)
        combine_input.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
        combine_input.isKeepToolBodies = False
        combine_feature = component.features.combineFeatures.add(combine_input)
        suffix = f'{layout.tooth_count}齿' if layout else '短榫'
        combine_feature.name = f'{label}-切割-{tool_index:02d}-{suffix}'
        if base_feature:
            created.append(base_feature)
        created.append(combine_feature)

    healthy = adsk.fusion.FeatureHealthStates.HealthyFeatureHealthState
    for feature in created:
        if feature.healthState != healthy:
            raise ValueError(f'自动箱形榫特征失败：{feature.errorOrWarningMessage}')
    for token, before_volume in before_volumes.items():
        body = adsk.fusion.BRepBody.cast(design.findEntityByToken(token)[0])
        actual_removed = before_volume - body.volume
        tolerance = max(0.001, expected_removed[token] * 0.02)
        if actual_removed <= 0 or abs(actual_removed - expected_removed[token]) > tolerance:
            raise ValueError(
                '标准切割的体积变化与榫槽工具体不一致；'
                f'预期 {expected_removed[token]:.6f} cm^3，'
                f'实际 {actual_removed:.6f} cm^3，'
                f'允许误差 {tolerance:.6f} cm^3。')
    return created


class AutoBoxJointAddIn:
    def __init__(self):
        self._handlers = []
        self._definition = None
        self._control_ids = []
        self._info = None
        self._face_plan = None
        self._mating_axis = None
        self._command = None
        self._preview_group = None
        self._preview_session = OverlapPreviewSession()
        self._selected_tokens = None
        self._preview_ready = False
        self._preview_creating = False
        self._preview_attempted = False
        self._preview_error = None
        app = adsk.core.Application.get()
        try:
            app.unregisterCustomEvent(RESTORE_EVENT_ID)
        except Exception:
            pass
        self._restore_event = app.registerCustomEvent(RESTORE_EVENT_ID)
        restore_handler = newEventHandler(
            self.on_restore_sources, adsk.core.CustomEventHandler)
        self._restore_event.add(restore_handler)
        self._handlers.append(restore_handler)
        design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
        if design:
            OverlapPreviewSession.recover_orphaned_state(design)
        self._create_command()

    def _create_command(self):
        ui = adsk.core.Application.get().userInterface
        old_definition = ui.commandDefinitions.itemById(COMMAND_ID)
        if old_definition:
            old_definition.deleteMe()
        self._definition = ui.commandDefinitions.addButtonDefinition(
            COMMAND_ID,
            '自动箱形榫',
            '选择两个相交实体，自动生成适合 3D 打印的箱形榫。',
            os.path.join(os.path.dirname(__file__), *command_resource_parts()),
        )
        handler = newEventHandler(self.on_create, adsk.core.CommandCreatedEventHandler)
        self._definition.commandCreated.add(handler)
        self._handlers.append(handler)

        for workspace_id, panel_id, promoted in toolbar_placements():
            panel = ui.workspaces.itemById(workspace_id).toolbarPanels.itemById(panel_id)
            old_control = panel.controls.itemById(COMMAND_ID)
            if old_control:
                old_control.deleteMe()
            control = panel.controls.addCommand(self._definition)
            control.isPromotedByDefault = promoted
            self._control_ids.append((workspace_id, panel_id))

    def stop(self):
        self._clear_preview()
        try:
            adsk.core.Application.get().unregisterCustomEvent(RESTORE_EVENT_ID)
        except Exception:
            handleException()
        self._restore_event = None
        ui = adsk.core.Application.get().userInterface
        for workspace_id, panel_id in self._control_ids:
            control = ui.workspaces.itemById(workspace_id).toolbarPanels.itemById(panel_id).controls.itemById(COMMAND_ID)
            if control:
                control.deleteMe()
        if self._definition:
            self._definition.deleteMe()
        self._handlers.clear()

    def on_create(self, event_args):
        command = event_args.command
        self._command = command
        self._info = None
        self._face_plan = None
        self._mating_axis = None
        self._selected_tokens = None
        self._preview_ready = False
        self._preview_creating = False
        self._preview_attempted = False
        self._preview_error = None
        inputs = command.commandInputs

        bodies = inputs.addSelectionInput('bodies', '相交实体', '选择两个具有体积相交的实体')
        bodies.addSelectionFilter('SolidBodies')
        # Fusion may drop one source selection after the selectable preview body
        # is created. Keep the UI input structurally valid and enforce the
        # initial two-body requirement in _selected_bodies/_refresh_state.
        bodies.setSelectionLimits(0, 2)
        locked_bodies = inputs.addTextBoxCommandInput(
            'lockedBodies', '相交实体', '已锁定两个相交实体。', 1, True)
        locked_bodies.isVisible = False

        mating_face = inputs.addSelectionInput('matingFace', '榫片配合面', '在半透明相交预览体上选择一个平面')
        mating_face.addSelectionFilter('PlanarFaces')
        mating_face.setSelectionLimits(0, 1)
        mating_face.isVisible = False
        locked_face = inputs.addTextBoxCommandInput(
            'lockedFace', '榫片配合面', '已锁定配合面。', 1, True)
        locked_face.isVisible = False
        inputs.addIntegerSpinnerCommandInput('toothCount', '榫片数量', 2, 99, 1, 4)
        inputs.addValueInput('clearance', '相邻榫片配合间隙', 'mm', adsk.core.ValueInput.createByString('0.20 mm'))
        inputs.addTextBoxCommandInput(
            'summary', '状态',
            '请选择两个相交实体；插件会生成可选择的半透明相交预览体。', 4, True)

        for event, callback, handler_type in (
            (command.inputChanged, self.on_input_changed, adsk.core.InputChangedEventHandler),
            (command.preSelect, self.on_pre_select, adsk.core.SelectionEventHandler),
            (command.validateInputs, self.on_validate_inputs, adsk.core.ValidateInputsEventHandler),
            (command.executePreview, self.on_preview, adsk.core.CommandEventHandler),
            (command.execute, self.on_execute, adsk.core.CommandEventHandler),
            (command.destroy, self.on_destroy, adsk.core.CommandEventHandler),
        ):
            handler = newEventHandler(callback, handler_type)
            event.add(handler)
            self._handlers.append(handler)

    def _selected_bodies(self, inputs):
        selection = inputs.itemById('bodies')
        if selection.selectionCount != 2:
            return None
        body_a = adsk.fusion.BRepBody.cast(selection.selection(0).entity)
        body_b = adsk.fusion.BRepBody.cast(selection.selection(1).entity)
        if not body_a or not body_b or body_a == body_b:
            return None
        return body_a, body_b

    @staticmethod
    def _face_normal_axis(face: adsk.fusion.BRepFace) -> int:
        normal = face.geometry.normal
        values = (abs(normal.x), abs(normal.y), abs(normal.z))
        return max(range(3), key=lambda index: values[index])

    def _refresh_state(self, inputs, allow_preview_creation=False):
        summary = inputs.itemById('summary')
        selected = self._selected_bodies(inputs)
        if not selected:
            if self._preview_creating:
                return
            body_input = inputs.itemById('bodies')
            if should_reset_body_selection_cache(
                    body_input.selectionCount,
                    self._preview_ready,
                    self._preview_creating):
                self._info = None
                self._selected_tokens = None
                self._preview_attempted = False
                self._preview_error = None
            if not self._info or not self._selected_tokens:
                summary.text = '请选择两个不同的实体。'
                return
        if selected:
            try:
                tokens = tuple(body.entityToken for body in selected)
                if tokens != self._selected_tokens:
                    self._clear_preview()
                    self._info = calculate_intersection(*selected)
                    self._selected_tokens = tokens
                    self._preview_ready = False
                    self._preview_attempted = False
                    self._preview_error = None
                    self._face_plan = None
                    self._mating_axis = None
                    inputs.itemById('matingFace').isVisible = False
                    inputs.itemById('lockedFace').isVisible = False
                    inputs.itemById('matingFace').clearSelection()
                if allow_preview_creation and should_create_overlap_preview(
                        2, self._preview_ready, self._preview_attempted):
                    self._preview_attempted = True
                    preview_created = False
                    self._preview_creating = True
                    try:
                        self._preview_session.create(self._info)
                        self._preview_ready = True
                        preview_created = True
                    except Exception as error:
                        self._preview_session.clear()
                        self._preview_error = str(error)
                        self._face_plan = None
                        summary.text = f'相交预览创建失败：{error}'
                        return
                    finally:
                        self._preview_creating = False
                    if preview_created:
                        inputs.itemById('bodies').isVisible = False
                        inputs.itemById('lockedBodies').isVisible = True
                        inputs.itemById('matingFace').isVisible = True
                if not self._preview_ready:
                    summary.text = self._preview_error or '已识别相交区，等待生成相交预览。'
                    return
            except ValueError as error:
                self._face_plan = None
                summary.text = str(error)
                return
        if not self._preview_ready:
            return
        try:
            face_input = inputs.itemById('matingFace')
            if self._mating_axis is None:
                if face_input.selectionCount != 1:
                    self._face_plan = None
                    summary.text = '请在半透明相交预览体上选择榫片配合面。'
                    return
                face = adsk.fusion.BRepFace.cast(face_input.selection(0).entity)
                if not self._preview_session.is_preview_face(face):
                    self._face_plan = None
                    summary.text = '配合面必须来自半透明相交预览体。'
                    return
                self._mating_axis = self._face_normal_axis(face)
                face_input.isVisible = False
                inputs.itemById('lockedFace').text = (
                    f'已锁定配合面；法线为 {("X", "Y", "Z")[self._mating_axis]}。')
                inputs.itemById('lockedFace').isVisible = True
            zero_extensions = ((0.0, 0.0), (0.0, 0.0), (0.0, 0.0))
            self._face_plan = plan_face_joint(FaceJointRequest(
                self._info.axis_lengths_mm, self._mating_axis, zero_extensions, zero_extensions,
                inputs.itemById('toothCount').value,
                inputs.itemById('clearance').value * 10.0,
            ))
            summary.text = self._face_plan.message
        except ValueError as error:
            self._face_plan = None
            summary.text = str(error)

    def on_input_changed(self, event_args):
        changed_input = event_args.input
        self._refresh_state(
            event_args.inputs,
            bool(changed_input and changed_input.id == 'bodies'),
        )

    def on_pre_select(self, event_args):
        active_input = event_args.activeInput
        active_input_id = active_input.id if active_input else ''
        entity = event_args.selection.entity
        event_args.isSelectable = selection_is_allowed(
            active_input_id,
            self._preview_session.is_preview_face(entity),
        )

    def on_validate_inputs(self, event_args):
        self._refresh_state(event_args.inputs)
        event_args.areInputsValid = bool(self._face_plan and self._face_plan.valid)

    def on_preview(self, event_args):
        inputs = event_args.command.commandInputs
        self._refresh_state(inputs)
        self._refresh_preview(inputs)
        # The preview contains only CustomGraphics, not the final cut features.
        # Keep this false so Fusion invokes on_execute when the user confirms.
        event_args.isValidResult = False

    def on_execute(self, event_args):
        self._refresh_state(event_args.command.commandInputs)
        if not self._face_plan or not self._face_plan.valid:
            event_args.executeFailed = True
            return
        tools = build_face_joint_tools(self._info, self._face_plan)
        self._release_source_selection(event_args.command.commandInputs)
        self._clear_preview()
        features = apply_standard_cuts(tools, label='自动箱形榫')
        event_args.command.commandInputs.itemById('summary').text = (
            f'已创建 {len(features)} 个原生特征；{self._face_plan.message}')
        event_args.executeFailed = False

    def on_destroy(self, _event_args):
        restore_tokens = tuple(self._selected_tokens or ())
        if self._command:
            self._release_source_selection(self._command.commandInputs)
        self._clear_preview()
        if restore_tokens:
            adsk.core.Application.get().fireCustomEvent(
                RESTORE_EVENT_ID, json.dumps(restore_tokens))
        self._command = None
        self._info = None
        self._face_plan = None
        self._mating_axis = None
        self._selected_tokens = None
        self._preview_ready = False
        self._preview_creating = False
        self._preview_attempted = False
        self._preview_error = None

    def on_restore_sources(self, event_args):
        """Run after command teardown so Fusion cannot reapply its picker lock."""
        tokens = json.loads(event_args.additionalInfo or '[]')
        design = adsk.fusion.Design.cast(
            adsk.core.Application.get().activeProduct)
        if design:
            for token in tokens:
                for entity in design.findEntityByToken(token):
                    body = adsk.fusion.BRepBody.cast(entity)
                    if body and body.isValid:
                        body.isSelectable = True
                        attribute = body.attributes.itemByName(
                            OverlapPreviewSession.STATE_GROUP,
                            OverlapPreviewSession.STATE_NAME)
                        if attribute:
                            attribute.deleteMe()
                        break

    @staticmethod
    def _release_source_selection(inputs):
        """Release Fusion's selection lock before restoring source bodies."""
        if not inputs:
            return
        body_input = inputs.itemById('bodies')
        if body_input:
            body_input.clearSelection()

    def _clear_preview(self):
        try:
            self._clear_graphics()
        finally:
            self._preview_session.clear()

    def _clear_graphics(self):
        if self._preview_group and self._preview_group.isValid:
            self._preview_group.deleteMe()
        self._preview_group = None

    def _refresh_preview(self, inputs):
        self._clear_graphics()
        if not self._info or not self._face_plan or not self._face_plan.valid:
            return
        tools = build_face_joint_tools(self._info, self._face_plan)
        design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
        self._preview_group = design.rootComponent.customGraphicsGroups.add()
        red = adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(220, 70, 60, 255))
        blue = adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(60, 130, 230, 255))
        for tool in tools:
            graphic = self._preview_group.addBRepBody(tool.body)
            graphic.color = red if tool.target_is_first_body else blue
            graphic.setOpacity(0.38, True)
