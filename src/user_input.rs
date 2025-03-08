// TODO:
// No support for multi-selection. Current Implementation is fairly imcompatible
// with multi-selection.

use bevy::color::palettes::css::RED;
use bevy::ecs::system::SystemParam;
use bevy::picking::mesh_picking::ray_cast::RayMeshHit;
use bevy::picking::pointer::{Location, PointerAction, PointerInput, PointerLocation,
                             PressDirection};
use bevy::prelude::*;
use bevy::render::mesh::MeshAabb;
use bevy::render::primitives::Aabb;
use bevy::utils::hashbrown::HashSet;
use bevy_rapier3d::dynamics::{RigidBody, RigidBodyDisabled};
use bevy_rapier3d::prelude::ExternalImpulse;

use crate::camera::MainCamera;

#[derive(Default, Resource)]
pub struct InputConfig {}

#[derive(SystemParam)]
pub struct WorldMeshPointerParams<'w, 's> {
	pub mesh_raycast: MeshRayCast<'w, 's>,
	pub main_camera:  Query<'w, 's, (&'static Camera, &'static GlobalTransform), With<MainCamera>>,
	pub pointers:     Query<'w, 's, &'static PointerLocation>,
}

impl<'w, 's> WorldMeshPointerParams<'w, 's> {
	pub fn get_pointer_hits(&mut self, rc_settings: &RayCastSettings) -> &[(Entity, RayMeshHit)] {
		let &mut WorldMeshPointerParams { ref mut mesh_raycast, ref main_camera, ref pointers } =
			self;

		let (cam, cam_global_t) = main_camera.single();
		let Some(Location { position, .. }) = pointers.single().location
			else { return &[] };

		let Ok(ray3d) = cam.viewport_to_world(cam_global_t, position)
			else { return &[] };

		mesh_raycast.cast_ray(ray3d, rc_settings)
	}
}

#[derive(Reflect, Clone, Copy, Debug)]
pub struct PointerFocusedObject {
	pub pointer_offset: Vec3,
	pub entity:         Entity,
}

#[derive(Reflect, Resource, Default)]
#[reflect(Resource, Default)]
pub struct InputState {
	/// Objects under influence by user input.
	pub focused:    Vec<PointerFocusedObject>,
	pub mode_state: InputModeState,
}

#[derive(Reflect, Clone, Copy)]
pub enum FlickPlane {
	XZPlane,
	ViewportPlane,
}

#[derive(Reflect, Clone, Copy)]
pub enum InputModeState {
	Dragging {
		/// Height above surface dragged onto.
		drag_height: f32,
	},
	Flicking {
		impulse_scale: f32,
		flick_plane:   FlickPlane,
	},
}

impl Default for InputModeState {
	fn default() -> Self {
		Self::Flicking { impulse_scale: 20., flick_plane: FlickPlane::ViewportPlane }
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
		.add_systems(Update, (cursor_drag, update_input_state, draw_flick_gizmo));
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
		return
	};

	let InputModeState::Dragging { drag_height } = r_input_state.mode_state
		else { return };

	let ents: HashSet<Entity> =
		r_input_state.focused.iter().map(|&PointerFocusedObject { entity, .. }| entity).collect();

	let Some(&(_, RayMeshHit { point, normal, .. })) = world_mesh
		.get_pointer_hits(&RayCastSettings {
			visibility: RayCastVisibility::VisibleInView,
			filter: &|e| !ents.contains(&e) && q_placeable.contains(e),
			..default()
		})
		.first()
		else { return };

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
				.filter(|&&(entity, _)| raycast_pickables.contains(entity))
				.next()
				else { return };

			let global_translation = global_transforms.get(entity).unwrap().translation();

			if let (Ok(rb), InputState { mode_state: InputModeState::Dragging { .. }, .. }) =
				(rigid_bodies.get(entity), &*r_input_state)
			{
				cmd.entity(entity).insert(PreviousRigidBody(rb.clone()));
				cmd.entity(entity).insert(RigidBody::Fixed);
				// NOTE:
				// This seems to cause bugs with Rapier.
				// cmd.entity(entity).remove::<RigidBody>();
			}

			r_input_state.focused =
				vec![PointerFocusedObject { pointer_offset: point - global_translation, entity }]
		},
		PointerAction::Pressed {
			direction: PressDirection::Up,
			button: PointerButton::Primary,
		} => match &*r_input_state {
			InputState {
				mode_state: InputModeState::Flicking { flick_plane, impulse_scale },
				..
			} => {
				let Some(point_pos) =
					world_mesh.pointers.single().location.as_ref().map(|x| x.position)
					else { return };
				
				let Some(&PointerFocusedObject { entity, pointer_offset }) =
					r_input_state.focused.first()
					else { return };

				let f_transl = global_transforms.get(entity).unwrap().translation();

				let Some(flick_vector) = get_flick_vector(
					*flick_plane,
					point_pos,
					f_transl + pointer_offset,
					world_mesh.main_camera.single(),
				)
					else { return };

				cmd.entity(entity).insert(ExternalImpulse::at_point(
					impulse_scale * flick_vector,
					f_transl + pointer_offset,
					f_transl,
				));

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
		_ => {},
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
	let InputState { mode_state: InputModeState::Flicking { flick_plane, .. }, focused } =
		&*r_input_state
		else { return };
	let Some(point_pos) = world_mesh.pointers.single().location.as_ref().map(|x| x.position)
		else { return };
	let Some(&PointerFocusedObject { entity, pointer_offset }) = focused.first()
		else { return };

	let f_transl = global_transforms.get(entity).unwrap().translation() + pointer_offset;

	let Some(flick_vector) =
		get_flick_vector(*flick_plane, point_pos, f_transl, world_mesh.main_camera.single())
		else { return };

	gizmos.arrow(f_transl - flick_vector, f_transl, RED);
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
