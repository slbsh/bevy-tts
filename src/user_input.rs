// TODO:
// No support for multi-selection. Current Implementation is fairly imcompatible
// with multi-selection.

use core::f32;

use bevy::color::palettes::css::{RED, YELLOW};
use bevy::ecs::system::SystemParam;
use bevy::math::Affine3A;
use bevy::math::bounding::{Aabb2d, Aabb3d, AabbCast3d};
use bevy::picking::mesh_picking::ray_cast::RayMeshHit;
use bevy::picking::pointer::{
	Location, PointerAction, PointerInput, PointerLocation, PressDirection,
};
use bevy::prelude::*;
use bevy::render::camera::{CameraProjection, SubCameraView};
use bevy::render::mesh::MeshAabb;
use bevy::render::primitives::{Aabb, Frustum};
use bevy::utils::hashbrown::HashSet;
use bevy_rapier3d::dynamics::RigidBody;
use bevy_rapier3d::prelude::ExternalImpulse;

use crate::camera::MainCamera;
use crate::util::{aabb_from_points, contains_obb};

pub const MAX_SELECTION_DISTANCE: f32 = 1000.;

#[derive(Resource)]
pub struct InputConfig {
	/// Height above surface dragged onto.
	drag_height: f32,
}

impl Default for InputConfig {
	fn default() -> Self {
		Self { drag_height: 1. }
	}
}

#[derive(SystemParam)]
pub struct WorldMeshPointerParams<'w, 's> {
	pub mesh_raycast: MeshRayCast<'w, 's>,
	pub main_camera: Query<'w, 's, (&'static Camera, &'static GlobalTransform), With<MainCamera>>,
	pub pointers: Query<'w, 's, &'static PointerLocation>,
}

impl<'w, 's> WorldMeshPointerParams<'w, 's> {
	pub fn get_pointer_hits(&mut self, rc_settings: &RayCastSettings) -> &[(Entity, RayMeshHit)] {
		let &mut WorldMeshPointerParams { ref mut mesh_raycast, ref main_camera, ref pointers } =
			self;

		let (cam, cam_global_t) = main_camera.single();
		let Some(Location { position, .. }) = pointers.single().location else { return &[] };

		let Ok(ray3d) = cam.viewport_to_world(cam_global_t, position) else { return &[] };

		mesh_raycast.cast_ray(ray3d, rc_settings)
	}
}

#[derive(Reflect, Clone, Copy, Debug)]
pub struct PointerFocusedObject {
	pub pointer_offset: Vec3,
	pub entity: Entity,
}

#[derive(Reflect, Resource, Default)]
#[reflect(Resource, Default)]
pub struct InputState {
	/// Objects under influence by user input.
	pub focused: Vec<PointerFocusedObject>,
	pub mode_state: InputModeState,
}

#[derive(Reflect, Clone, Copy)]
pub enum FlickPlane {
	XZPlane,
	ViewportPlane,
}

#[derive(Reflect, Clone, Copy)]
pub enum InputModeState {
	Grab,
	Select { initial_position: Option<Vec2> },
	Flick { impulse_scale: f32, flick_plane: FlickPlane },
}

impl Default for InputModeState {
	fn default() -> Self {
		// Self::Flick { impulse_scale: 20., flick_plane: FlickPlane::XZPlane }
		Self::Select { initial_position: None }
	}
}

/// Marker Component for Entities that can have objects place ontop of them (used in mouse raycast)
#[derive(Component)]
pub struct PlacablePlatform;

#[derive(Component)]
pub struct PreviousRigidBody(pub RigidBody);

/// Marker Component for Entities that can't be moved with user input
#[derive(Component)]
pub struct Locked;

pub const MOUSE_RAY_TOI: f32 = 1000.;

pub fn plugin(app: &mut App) {
	app.init_resource::<InputConfig>().init_resource::<InputState>().add_systems(
		Update,
		(cursor_drag, update_input_state, draw_flick_gizmo, select_region, draw_selected_gizmo),
	);
}

fn cursor_drag(
	mut world_mesh: WorldMeshPointerParams,
	mut transforms: Query<&mut Transform>,
	r_input_state: Res<InputState>,
	r_input_config: Res<InputConfig>,
	q_mesh: Query<&Mesh3d>,
	r_mesh: Res<Assets<Mesh>>,
	q_placeable: Query<(), With<PlacablePlatform>>,
) {
	let (drag_height, focused @ [_, ..], InputModeState::Grab) =
		(r_input_config.drag_height, &r_input_state.focused[..], r_input_state.mode_state)
	else {
		return;
	};

	let ents: HashSet<Entity> =
		focused.iter().map(|&PointerFocusedObject { entity, .. }| entity).collect();

	let Some(&(_, RayMeshHit { point, normal, .. })) = world_mesh
		.get_pointer_hits(&RayCastSettings {
			visibility: RayCastVisibility::VisibleInView,
			filter: &|e| !ents.contains(&e) && q_placeable.contains(e),
			..default()
		})
		.first()
	else {
		return;
	};

	focused.iter().for_each(|&PointerFocusedObject { entity, pointer_offset }| {
		let &Transform { rotation, scale, .. } = transforms.get(entity).unwrap();

		let mut transform = transforms.get_mut(entity).unwrap();
		let Mesh3d(mesh_h) = q_mesh.get(entity).unwrap();
		let mesh = r_mesh.get(mesh_h).unwrap();

		// TODO: In general, likely needs to use Aabb center.
		let Aabb { half_extents, .. } = mesh.compute_aabb().unwrap();

		*transform = Transform {
			translation: point + (half_extents.y + drag_height) * normal,
			// + (pointer_offset - pointer_offset.dot(normal) * normal),
			rotation: Quat::from_axis_angle(normal, rotation.to_axis_angle().1),
			scale,
		};
	});
}

// FIXME: Monster function
fn update_input_state(
	mut cmd: Commands,
	mut e_pointer_input: EventReader<PointerInput>,
	mut world_mesh: WorldMeshPointerParams,
	mut r_input_state: ResMut<InputState>,
	rigid_bodies: Query<&RigidBody>,
	raycast_pickables: Query<&RayCastPickable>,
	previous_rigid_bodies: Query<&PreviousRigidBody>,
	global_transforms: Query<&GlobalTransform, Without<MainCamera>>,
) {
	e_pointer_input.read().for_each(|PointerInput { action, .. }| match action {
		PointerAction::Pressed {
			direction: PressDirection::Down,
			button: PointerButton::Primary,
		} => {
			let Some(&(entity, RayMeshHit { point, .. })) = world_mesh
				.get_pointer_hits(&RayCastSettings {
					visibility: RayCastVisibility::VisibleInView,
					..default()
				})
				.iter()
				.find(|&&(entity, _)| raycast_pickables.contains(entity))
			else {
				if let (Some(initial_position), InputModeState::Select { .. }) = (
					world_mesh.pointers.single().location.as_ref().map(|x| x.position),
					r_input_state.mode_state,
				) {
					r_input_state.mode_state =
						InputModeState::Select { initial_position: Some(initial_position) };
				}

				return;
			};

			let global_translation = global_transforms.get(entity).unwrap().translation();

			if let (Ok(rb), InputModeState::Select { .. }) =
				(rigid_bodies.get(entity), r_input_state.mode_state)
			{
				cmd.entity(entity).insert(PreviousRigidBody(*rb));
				cmd.entity(entity).insert(RigidBody::Fixed);
				// NOTE:
				// This seems to cause bugs with Rapier.
				// cmd.entity(entity).remove::<RigidBody>();
			}

			// If in Select mode, replaces the focus to a single object, unless holding `Shift` `Alt`.

			if matches!(r_input_state.mode_state, InputModeState::Select { .. }) {
				r_input_state.mode_state = InputModeState::Grab;
			}

			r_input_state
				.focused
				= vec![PointerFocusedObject { pointer_offset: point - global_translation, entity }]
		},
		PointerAction::Pressed {
			direction: PressDirection::Down,
			button: PointerButton::Secondary,
		} => {
			if matches!(r_input_state.mode_state, InputModeState::Select { .. }) {
				r_input_state.focused = vec![];
			}
		},
		PointerAction::Pressed {
			direction: PressDirection::Up,
			button: PointerButton::Primary,
		} => match &*r_input_state {
			InputState { mode_state: InputModeState::Select { .. }, .. } => {
				r_input_state.mode_state = InputModeState::Select { initial_position: None };
			},
			InputState {
				mode_state: InputModeState::Flick { flick_plane, impulse_scale }, ..
			} => {
				let Some(point_pos) =
					world_mesh.pointers.single().location.as_ref().map(|x| x.position)
				else {
					return;
				};

				let Some(&PointerFocusedObject { entity, pointer_offset }) =
					r_input_state.focused.first()
				else {
					return;
				};

				let f_transl = global_transforms.get(entity).unwrap().translation();

				let Some(flick_vector) = get_flick_vector(
					*flick_plane,
					point_pos,
					f_transl + pointer_offset,
					world_mesh.main_camera.single(),
				) else {
					return;
				};

				cmd.entity(entity).insert(ExternalImpulse::at_point(
					impulse_scale * flick_vector,
					f_transl + pointer_offset,
					f_transl,
				));

				r_input_state
					.focused
					= vec![];
				},
			InputState { mode_state: InputModeState::Grab { .. }, .. } => {
				r_input_state.focused.iter().for_each(
					|&PointerFocusedObject { entity, .. }| {
						if let Ok(PreviousRigidBody(rb)) = previous_rigid_bodies.get(entity) {
							cmd.entity(entity).insert(*rb);
							cmd.entity(entity).remove::<PreviousRigidBody>();
						}
					},
				);

				r_input_state.mode_state = InputModeState::Select { initial_position: None };
			},
		},
		_ => {},
	});
}

// TODO:
// Maybe use (https://github.com/komadori/bevy_mod_outline)
fn draw_selected_gizmo(
	mut gizmos: Gizmos,
	aabbs: Query<(&Aabb, &GlobalTransform)>,
	r_input_state: Res<InputState>,
) {
	r_input_state.focused.iter().for_each(|&PointerFocusedObject { entity, .. }| {
		let Ok((aabb, global_transform)) = aabbs.get(entity) else { return };
		gizmos.cuboid(
			global_transform.compute_transform().with_scale(2. * Vec3::from(aabb.half_extents)),
			YELLOW,
		);
	});
}

fn draw_flick_gizmo(
	mut gizmos: Gizmos,
	world_mesh: WorldMeshPointerParams,
	global_transforms: Query<&GlobalTransform, Without<MainCamera>>,
	r_input_state: Res<InputState>,
) {
	// FIXME:
	// In future when you can flick a group of selected objects,
	// this will need to change
	let InputState { mode_state: InputModeState::Flick { flick_plane, .. }, focused } =
		&*r_input_state
	else {
		return;
	};
	let Some(point_pos) = world_mesh.pointers.single().location.as_ref().map(|x| x.position) else {
		return;
	};
	let Some(&PointerFocusedObject { entity, pointer_offset }) = focused.first() else { return };

	let f_transl = global_transforms.get(entity).unwrap().translation() + pointer_offset;

	let Some(flick_vector) =
		get_flick_vector(*flick_plane, point_pos, f_transl, world_mesh.main_camera.single())
	else {
		return;
	};

	gizmos.arrow(f_transl - flick_vector, f_transl, RED);
}

#[derive(Component)]
struct SelectionBoxBoi;

fn select_region(
	mut cmd: Commands,
	mut r_input_state: ResMut<InputState>,
	mut select_box_boi: Option<Single<(Entity, &mut Node), With<SelectionBoxBoi>>>,
	pointers: Query<&PointerLocation>,
	visible_objects: Query<
		(Entity, &GlobalTransform, &Aabb, &InheritedVisibility),
		(With<RayCastPickable>, Without<MainCamera>),
	>,
	main_camera: Query<(&GlobalTransform, &Camera, &Projection), With<MainCamera>>,
) {
	let (InputModeState::Select { initial_position: Some(initial_position) }, Some(point_pos)) =
		(r_input_state.mode_state, pointers.single().location.as_ref().map(|x| x.position))
	else {
		if let Some((boyo, _)) = select_box_boi.map(|x| x.into_inner()) {
			cmd.entity(boyo).despawn();
		}
		return;
	};

	let Aabb2d { min: min @ Vec2 { x: a_x, y: a_y }, max: Vec2 { x: b_x, y: b_y } } =
		aabb_from_points(initial_position, point_pos);

	// Guard when the area of the box is zero.
	if (b_x - a_x) * (b_y - a_x) == 0. {
		if let Some((boyo, _)) = select_box_boi.map(|x| x.into_inner()) {
			cmd.entity(boyo).despawn();
		}
		return;
	};

	let selection_box_size = UVec2 { x: (b_x - a_x) as u32, y: (b_y - a_y) as u32 };

	if let Some((_, mut node)) = select_box_boi.map(|x| x.into_inner()) {
		let UVec2 { x, y } = selection_box_size;

		*node = Node {
			top: Val::Px(a_y),
			left: Val::Px(a_x),
			width: Val::Px(x as f32),
			height: Val::Px(y as f32),
			..node.clone()
		}
	} else {
		let UVec2 { x, y } = selection_box_size;
		cmd.spawn((
			Node {
				top: Val::Px(a_y),
				left: Val::Px(a_x),
				width: Val::Px(x as f32),
				height: Val::Px(y as f32),
				position_type: PositionType::Absolute,
				..default()
			},
			BackgroundColor(Color::srgba(0.5, 0.8, 1., 0.7)),
			SelectionBoxBoi,
		));
	}

	let (cam_transform, camera, projection) = main_camera.single();

	let target_size = camera.physical_viewport_size().unwrap();

	let sub_cam = projection.get_clip_from_view_for_sub(&SubCameraView {
		full_size: target_size,
		offset: min,
		size: selection_box_size,
	});

	let sub_frustum = Frustum::from_clip_from_world(&sub_cam);

	// TODO:
	// Compute that the overlapping AABB is above some threshold, rather than
	// simply checking if it intersects or not.
	r_input_state.focused = visible_objects
		.iter()
		.filter_map(|(entity, global_transform, aabb, &visibility)| {
			if visibility == InheritedVisibility::VISIBLE
				&& contains_obb(
					&sub_frustum,
					aabb,
					&Affine3A::from_mat4(
						cam_transform.compute_matrix().inverse()
							* global_transform.compute_matrix(),
					),
					// FIXME: Should it be true of false ?
					true,
					true,
				) {
				Some(PointerFocusedObject { pointer_offset: Vec3::ZERO, entity })
			} else {
				None
			}
		})
		.collect();
}

fn get_flick_vector(
	flick_plane: FlickPlane,
	pointer_position: Vec2,
	focus_point: Vec3,
	(main_camera, cam_transform): (&Camera, &GlobalTransform),
) -> Option<Vec3> {
	let vp_ray3d = main_camera.viewport_to_world(cam_transform, pointer_position).ok()?;

	let inf_plane = match flick_plane {
		FlickPlane::ViewportPlane => InfinitePlane3d { normal: -vp_ray3d.direction },
		FlickPlane::XZPlane => InfinitePlane3d { normal: Dir3::Y },
	};

	let distance = vp_ray3d.intersect_plane(focus_point, inf_plane)?;

	Some(focus_point - (vp_ray3d.origin + distance * vp_ray3d.direction))
}
