use bevy::color::palettes::css::RED;
use bevy::ecs::system::SystemParam;
use bevy::picking::mesh_picking::ray_cast::RayMeshHit;
use bevy::picking::pointer::{
	Location, PointerAction, PointerInput, PointerLocation, PressDirection,
};
use bevy::prelude::*;
use bevy::render::mesh::MeshAabb;
use bevy::render::primitives::Aabb;
use bevy::utils::hashbrown::HashSet;
use bevy_rapier3d::dynamics::{RigidBody, RigidBodyDisabled};

use crate::camera::MainCamera;

#[derive(Default, Resource)]
pub struct InputConfig {}

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
		let Ok(ray3d) = cam.viewport_to_world(cam_global_t, position) else {
			return &[];
		};

		mesh_raycast.cast_ray(ray3d, rc_settings)
	}
}

#[derive(Clone, Copy, Debug)]
pub struct PointerFocusedObject {
	pub pointer_offset: Vec3,
	pub entity: Entity,
}

#[derive(Resource, Default)]
pub struct InputState {
	/// Objects under influence by user input.
	pub focused: Vec<PointerFocusedObject>,
	pub mode_state: InputModeState,
}

#[derive(Clone, Copy)]
pub enum InputModeState {
	Dragging {
		/// Height above surface dragged onto.
		drag_height: f32,
	},
	Flicking {
		impulse_scale: f32,
	},
}

impl Default for InputModeState {
	fn default() -> Self {
		Self::Flicking { impulse_scale: 1. }
		// Self::Dragging { drag_height: 1. }
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
	app.init_resource::<InputConfig>()
		.init_resource::<InputState>()
		.add_systems(Update, (cursor_drag, update_input_state));
}

fn cursor_drag(
	mut world_mesh: WorldMeshPointerParams,
	mut transforms: Query<&mut Transform>,
	r_input_state: Res<InputState>,
	q_mesh: Query<&Mesh3d>,
	r_mesh: Res<Assets<Mesh>>,
	q_placeable: Query<(), With<PlacablePlatform>>,
) {
	if r_input_state.focused.is_empty() {
		return;
	};

	let InputModeState::Dragging { drag_height } = r_input_state.mode_state else {
		return;
	};

	let ents: HashSet<Entity> =
		r_input_state.focused.iter().map(|&PointerFocusedObject { entity, .. }| entity).collect();

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

	r_input_state.focused.iter().for_each(|&PointerFocusedObject { entity, pointer_offset }| {
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
	mut gizmos: Gizmos,
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
			match &*r_input_state {
				InputState { mode_state: InputModeState::Flicking { .. }, .. } => {
					let Some(&(entity, RayMeshHit { point, .. })) = world_mesh
						.get_pointer_hits(&RayCastSettings {
							visibility: RayCastVisibility::VisibleInView,
							..default()
						})
						.iter()
						.filter(|&&(entity, _)| raycast_pickables.contains(entity))
						.next()
					else {
						return;
					};

					let global_translation = global_transforms.get(entity).unwrap().translation();

					r_input_state.focused = vec![PointerFocusedObject {
						pointer_offset: global_translation - point,
						entity,
					}]
				},
				InputState { mode_state: InputModeState::Dragging { .. }, .. } => {
					let Some(&(entity, RayMeshHit { point, .. })) = world_mesh
						.get_pointer_hits(&RayCastSettings {
							visibility: RayCastVisibility::VisibleInView,
							..default()
						})
						.iter()
						.filter(|&&(entity, _)| raycast_pickables.contains(entity))
						.next()
					else {
						return;
					};

					let global_translation = global_transforms.get(entity).unwrap().translation();

					if let Ok(rb) = rigid_bodies.get(entity) {
						cmd.entity(entity).insert(PreviousRigidBody(rb.clone()));
						cmd.entity(entity).insert(RigidBody::Fixed);
						// cmd.entity(entity).remove::<RigidBody>();
					}

					r_input_state.focused = vec![PointerFocusedObject {
						pointer_offset: global_translation - point,
						entity,
					}]
				},
			}
		},
		PointerAction::Pressed {
			direction: PressDirection::Up,
			button: PointerButton::Primary,
		} => match &*r_input_state {
			InputState { mode_state: InputModeState::Flicking { .. }, .. } => {
				r_input_state.focused = vec![];
			},
			InputState { mode_state: InputModeState::Dragging { .. }, .. } => {
				r_input_state.focused.iter().for_each(
					|&PointerFocusedObject { entity, .. }| {
						if let Ok(PreviousRigidBody(rb)) = previous_rigid_bodies.get(entity) {
							cmd.entity(entity).insert(rb.clone());
							cmd.entity(entity).remove::<PreviousRigidBody>();
						}
					},
				);

				r_input_state.focused = vec![];
			},
		},
		PointerAction::Moved { .. } => match &*r_input_state {
			// FIXME:
			// In future when you can flick a group of selected objects,
			// this will need to change
			InputState { mode_state: InputModeState::Flicking { .. }, .. } => {
				let Some(point_pos) = &world_mesh.pointers.single().location else { return };
				let Some(&PointerFocusedObject { entity, .. }) = r_input_state.focused.first()
				else {
					return;
				};

				let (main_camera, cam_transform) = world_mesh.main_camera.single();
				let focused_transform = global_transforms.get(entity).unwrap();
				let f_transl @ Vec3 { x: f_transl_x, y: f_transl_y, z: f_transl_z } =
					focused_transform.translation();

				let Ok(vp_ray3d) = main_camera.viewport_to_world(cam_transform, point_pos.position)
				else {
					return;
				};

				// TODO:
				// Also try a plane that's parallel to the viewport (simply `vp_ray3d.direction`)
				let distance = vp_ray3d
					.intersect_plane(f_transl_y * Vec3::Y, InfinitePlane3d { normal: Dir3::Y })
					.unwrap();

				let intersection_point = vp_ray3d.origin + distance * vp_ray3d.direction;
				gizmos.arrow(intersection_point, f_transl, RED);
			},
			_ => {},
		},
		_ => {},
	});
}
