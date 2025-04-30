// TODO:
// Multi-dragging needs the following things done :
//	- All bodies selected become `Static`
//	- Maybe move group of objects relative to
//	  the object clicked & dragged ?
//	- pointer_offset from click_drag isn't very nice
//	  on single object selections (because the offset point is
//	  set to the distance from the center of the object to the
//	  point on the surface that the pointer hit)

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
	drag_spread: f32,
}

// TODO:
// Change to a HashMap for PointerIds. Assumes 1 poitner.
#[derive(Resource, Default, Deref)]
pub struct LastPointerLocation(pub PointerLocation);

impl Default for InputConfig {
	fn default() -> Self {
		Self { drag_height: 1., drag_spread: 2. }
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

impl FlickPlane {
	pub fn to_infinite_plane(&self, vp_ray3d: Ray3d) -> InfinitePlane3d {
		match self {
			FlickPlane::ViewportPlane => InfinitePlane3d { normal: -vp_ray3d.direction },
			FlickPlane::XZPlane => InfinitePlane3d { normal: Dir3::Y },
		}
	}
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

#[derive(Component, Default)]
pub struct PreviousRigidBody(pub RigidBody);

/// Marker Component for Entities that can't be moved with user input
#[derive(Component)]
pub struct Locked;

pub const MOUSE_RAY_TOI: f32 = 1000.;

pub fn plugin(app: &mut App) {
	app.init_resource::<InputConfig>().init_resource::<InputState>()
		.init_resource::<LastPointerLocation>()
		.add_systems(
			Update,
			(cursor_drag, update_input_state, draw_flick_gizmo, select_region, draw_selected_gizmo),
	).add_systems(PostUpdate, last_pointer_update);
}

fn last_pointer_update(mut r: ResMut<LastPointerLocation>, q: Query<&PointerLocation>,) {
	if let Ok(p) = q.get_single() {
		r.0 = p.clone();
	}
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
	let (InputConfig { drag_height, drag_spread }, focused @ [_, ..], InputModeState::Grab) =
		(&*r_input_config, &r_input_state.focused[..], r_input_state.mode_state)
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

	let max_offset : f32 = focused.iter().fold(0., |a, pfo| a.max(pfo.pointer_offset.length()));

	focused.iter().for_each(|&PointerFocusedObject { entity, pointer_offset }| {
		let &Transform { rotation, scale, .. } = transforms.get(entity).unwrap();

		let mut transform = transforms.get_mut(entity).unwrap();
		let Mesh3d(mesh_h) = q_mesh.get(entity).unwrap();
		let mesh = r_mesh.get(mesh_h).unwrap();

		// TODO: In general, likely needs to use Aabb center.
		let Aabb { half_extents, .. } = mesh.compute_aabb().unwrap();

		*transform = Transform {
			translation: point + (Vec3::from(half_extents).dot(normal.abs()) + drag_height) * normal + drag_spread * pointer_offset / max_offset,
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
	r_last_pointer_position: Res<LastPointerLocation>,
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

			// FIXME:
			// Currently only checks if Dynamic, which means Kinematic bodies will lose their Kinematic
			// body type.
			if let (Ok(rb@RigidBody::Dynamic), InputModeState::Select { .. }) =
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

			if !r_input_state.focused.iter().find(|pfo| pfo.entity == entity).is_some() {
				r_input_state.focused =
					vec![PointerFocusedObject { pointer_offset: point - global_translation, entity }]
			}
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
			InputState { mode_state: InputModeState::Select { .. }, focused } => {
				focused.iter().for_each(|&PointerFocusedObject {entity, ..}| {
					// FIXME:
					// Currently only checks if Dynamic, which means Kinematic bodies will lose their Kinematic
					// body type.
					if let (Ok(rb@RigidBody::Dynamic), InputModeState::Select { .. }) =
						(rigid_bodies.get(entity), r_input_state.mode_state)
					{
						cmd.entity(entity).insert(PreviousRigidBody(*rb));
						cmd.entity(entity).insert(RigidBody::Fixed);
						// NOTE:
						// This seems to cause bugs with Rapier.
						// cmd.entity(entity).remove::<RigidBody>();
					}
				});

				r_input_state.mode_state = InputModeState::Select { initial_position: None };
			},
			InputState {
				mode_state: InputModeState::Flick { flick_plane, impulse_scale }, ..
			} => {
				// NOTE:
				// I know you love it, Anthony
				if let Some((entity, f_transl, pointer_offset, flick_vector)) = world_mesh.pointers.single().location.as_ref().and_then(|&Location { position, .. }| {
					let &PointerFocusedObject { entity, pointer_offset } = r_input_state.focused.first()?;
					let f_transl = global_transforms.get(entity).map(|gt| gt.translation()).ok()?;
					let flick_vector = world_mesh.main_camera.get_single().ok()
						.and_then(|(main_camera, cam_transform)| main_camera.viewport_to_world(cam_transform, position).ok())
						.and_then(|vp_ray3d| get_flick_vector(*flick_plane, vp_ray3d, f_transl))?;

					Some((entity, f_transl, pointer_offset, flick_vector))
				}) {
					cmd.entity(entity).insert(ExternalImpulse::at_point(
						impulse_scale * flick_vector,
						f_transl + pointer_offset,
						f_transl,
					));

					r_input_state.focused = vec![];
				}
			},
			InputState { mode_state: InputModeState::Grab { .. }, .. } => {
				r_input_state.focused.iter().for_each(
					|&PointerFocusedObject { entity, .. }| {
						if let Ok(PreviousRigidBody(rb)) = previous_rigid_bodies.get(entity) {
							cmd.entity(entity).insert(*rb);
							cmd.entity(entity).remove::<PreviousRigidBody>();
						};
						if let Some(flick_vector) = world_mesh.pointers.get_single().ok().and_then(|p| {
							let pointer_position = p.location.as_ref().map(|x| x.position)?.reflect(Vec2::Y);
							let lpp = r_last_pointer_position.location.as_ref().map(|x| x.position)?.reflect(Vec2::Y);
							let (_, &m_cam_transform) = world_mesh.main_camera.get_single().ok()?;

							let delta_p = pointer_position - lpp;

							Some(m_cam_transform.affine().transform_vector3(Vec3 {x: delta_p.x, y: 0., z: - delta_p.y}))
						}) {
							cmd.entity(entity).insert(ExternalImpulse {
								impulse: 10. * flick_vector,..Default::default()
							}
							);
						}
					},
				);

				r_input_state.focused = vec![];

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

	let Some(flick_vector) = world_mesh.main_camera.get_single().ok()
		.and_then(|(main_camera, cam_transform)| main_camera.viewport_to_world(cam_transform, point_pos).ok())
		.and_then(|vp_ray3d| get_flick_vector(*flick_plane, vp_ray3d, f_transl))
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
	select_box_boi: Option<Single<(Entity, &mut Node), With<SelectionBoxBoi>>>,
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

	let selected_objs : Vec<(Vec3, Entity)> = visible_objects
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
					// FIXME: Should it be true or false ?
					true,
					true,
				) {
				Some(( global_transform.translation(), entity ))
			} else {
				None
			}
		})
		.collect();

	let center_point = selected_objs.iter().fold(Vec3::ZERO, |a, &(vb, _)| a + vb) / (selected_objs.len() as f32);

	r_input_state.focused = selected_objs.iter().map(|&(pos, entity)| PointerFocusedObject {
		pointer_offset: pos - center_point,
		entity
	}).collect()
}

fn get_flick_vector(
	flick_plane: FlickPlane,
	vp_ray3d@Ray3d { origin, direction }: Ray3d,
	focus_point: Vec3,
) -> Option<Vec3> {
	let inf_plane = flick_plane.to_infinite_plane(vp_ray3d);

	let distance = vp_ray3d.intersect_plane(focus_point, inf_plane)?;

	Some(focus_point - (origin + distance * direction))
}
