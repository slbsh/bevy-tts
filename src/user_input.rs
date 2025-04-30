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
use bevy::math::bounding::Aabb2d;
use bevy::picking::mesh_picking::ray_cast::RayMeshHit;
use bevy::picking::pointer::{
	PointerAction, PointerInput, PointerLocation, PressDirection,
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
		let (cam, cam_global_t) = self.main_camera.single();

		self.pointers.single().location.as_ref()
			.and_then(|l| cam.viewport_to_world(cam_global_t, l.position).ok())
			.map_or(&[], |r| self.mesh_raycast.cast_ray(r, rc_settings))
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

#[derive(Reflect, Default, Clone, Copy)]
#[reflect(Default)]
pub enum FlickPlane {
	#[default]
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
#[reflect(Default)]
pub enum InputModeState {
	Grab,
	Select { initial_position: Option<Vec2> },
	Flick { impulse_scale: f32, flick_plane: FlickPlane },
}

impl Default for InputModeState {
	fn default() -> Self {
		 Self::Flick { impulse_scale: 20., flick_plane: Default::default() }
		//Self::Select { initial_position: None }
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
	if matches!(r_input_state.mode_state, InputModeState::Grab) { return }

	let Some(focused) = r_input_state.focused.get(1..) else { return };

	let ents: HashSet<Entity> = focused.iter().map(|&p| p.entity).collect();

	let Some(&(_, RayMeshHit { point, normal, .. })) = world_mesh
		.get_pointer_hits(&RayCastSettings {
			visibility: RayCastVisibility::VisibleInView,
			filter: &|e| !ents.contains(&e) && q_placeable.contains(e),
			..default()
		})
		.first()
		else { return };

	let max_offset: f32 = focused.iter().fold(0., |a, pfo| a.max(pfo.pointer_offset.length()));

	focused.iter().for_each(|p| {
		let &Transform { rotation, scale, .. } = transforms.get(p.entity).unwrap();

		let mut transform = transforms.get_mut(p.entity).unwrap();
		let Mesh3d(mesh_h) = q_mesh.get(p.entity).unwrap();
		let mesh = r_mesh.get(mesh_h).unwrap();

		// TODO: In general, likely needs to use Aabb center.
		let Aabb { half_extents, .. } = mesh.compute_aabb().unwrap();

		*transform = Transform {
			translation: point + (Vec3::from(half_extents).dot(normal.abs()) + r_input_config.drag_height) 
				* normal + r_input_config.drag_spread * p.pointer_offset / max_offset,
			// + (pointer_offset - pointer_offset.dot(normal) * normal),
			rotation: Quat::from_axis_angle(normal, rotation.to_axis_angle().1),
			scale,
		};
	});
}

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
	e_pointer_input.read().for_each(|i| match i.action {
		PointerAction::Pressed {
			direction: PressDirection::Down,
			button: PointerButton::Primary,
		} => {
			let Some(&(entity, RayMeshHit { point, .. })) = world_mesh
				.get_pointer_hits(&RayCastSettings::default()
					.with_visibility(RayCastVisibility::VisibleInView))
				.iter()
				.find(|&&(e, _)| raycast_pickables.contains(e))
			else {
				if let Some(loc) = &world_mesh.pointers.single().location
					&& let InputModeState::Select { initial_position } = &mut r_input_state.mode_state {
					*initial_position = Some(loc.position);
				}

				return;
			};

			let global_translation = global_transforms.get(entity).unwrap().translation();

			// FIXME: Currently only checks if Dynamic, which means 
			// Kinematic bodies will lose their Kinematic body type.
			if matches!(r_input_state.mode_state, InputModeState::Select { .. }) {
				if let Ok(rb @ RigidBody::Dynamic) = rigid_bodies.get(entity) {
					cmd.entity(entity).insert(PreviousRigidBody(*rb));
					cmd.entity(entity).insert(RigidBody::Fixed);
					// NOTE: This seems to cause bugs with Rapier.
					// cmd.entity(entity).remove::<RigidBody>();
				}

				// If in Select mode, replaces the focus to a single object, unless holding `Shift` `Alt`.
				r_input_state.mode_state = InputModeState::Grab;
			}

			if !r_input_state.focused.iter().find(|pfo| pfo.entity == entity).is_some() {
				r_input_state.focused =
					vec![PointerFocusedObject { pointer_offset: point - global_translation, entity }]
			}
		},

		PointerAction::Pressed { direction: PressDirection::Down, button: PointerButton::Secondary }
			if matches!(r_input_state.mode_state, InputModeState::Select { .. }) => r_input_state.focused.clear(),

		PointerAction::Pressed {
			direction: PressDirection::Up,
			button: PointerButton::Primary,
		} => match r_input_state.mode_state {
			InputModeState::Select { .. } => {
				r_input_state.focused.iter().for_each(|p| {
					if let Ok(rb @ RigidBody::Dynamic) = rigid_bodies.get(p.entity) {
						cmd.entity(p.entity).insert(PreviousRigidBody(*rb));
						cmd.entity(p.entity).insert(RigidBody::Fixed);
						// NOTE: This seems to cause bugs with Rapier.
						// cmd.entity(entity).remove::<RigidBody>();
					}
				});

				r_input_state.mode_state = InputModeState::Select { initial_position: None };
			},
			InputModeState::Flick { flick_plane, impulse_scale } => {
				let Some(focused) = r_input_state.focused.first() else { return };

				let Ok(f_transl) = global_transforms.get(focused.entity)
					.map(|gt| gt.translation()) else { return };

				let Some(ref ptr_loc) = world_mesh.pointers.single().location else { return };

				let Some(flick_vector) = world_mesh.main_camera.get_single().ok()
					.and_then(|(mc, ct)| mc.viewport_to_world(ct, ptr_loc.position).ok())
					.and_then(|vp_ray3d| get_flick_vector(flick_plane, vp_ray3d, f_transl)) else { return };

				cmd.entity(focused.entity).insert(ExternalImpulse::at_point(
					impulse_scale * flick_vector,
					f_transl + focused.pointer_offset,
					f_transl,
				));

				r_input_state.focused.clear();
			},

			InputModeState::Grab { .. } => {
				r_input_state.focused.iter().for_each(|&p| {
					if let Ok(PreviousRigidBody(rb)) = previous_rigid_bodies.get(p.entity) {
						cmd.entity(p.entity).insert(*rb);
						cmd.entity(p.entity).remove::<PreviousRigidBody>();
					};

					let Ok(ptr) = world_mesh.pointers.get_single() else { return };

					let Some(ptr_pos) = ptr.location.as_ref().map(|x| x.position.reflect(Vec2::Y)) else { return };
					let Some(lpp) = r_last_pointer_position.location.as_ref().map(|x| x.position.reflect(Vec2::Y)) else { return };

					let Ok((_, &m_cam_transform)) = world_mesh.main_camera.get_single() else { return };

					// x // y
					let delta_p = ptr_pos - lpp;

					cmd.entity(p.entity).insert(ExternalImpulse {
						impulse: 10. * m_cam_transform.affine().transform_vector3(Vec3::new(delta_p.x, 0., -delta_p.y)),
						..Default::default()
					});
				});

				r_input_state.focused.clear();
				r_input_state.mode_state = InputModeState::Select { initial_position: None };
			}
		},
		_ => {},
	});
}

// TODO: Maybe use (https://github.com/komadori/bevy_mod_outline)
fn draw_selected_gizmo(
	mut gizmos: Gizmos,
	aabbs: Query<(&Aabb, &GlobalTransform)>,
	r_input_state: Res<InputState>,
) {
	r_input_state.focused.iter().for_each(|p| {
		let Ok((aabb, global_transform)) = aabbs.get(p.entity) else { return };
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
	// FIXME: In future when you can flick a group of selected objects, this will need to change
	let InputState { mode_state: InputModeState::Flick { flick_plane, .. }, focused } = &*r_input_state
		else { return };

	let Some(ref point_loc) = world_mesh.pointers.single().location else { return };
	let Some(&PointerFocusedObject { entity, pointer_offset }) = focused.first() else { return };

	let f_transl = global_transforms.get(entity).unwrap().translation() + pointer_offset;

	let Some(flick_vector) = world_mesh.main_camera.get_single().ok()
		.and_then(|(main_camera, cam_transform)| main_camera.viewport_to_world(cam_transform, point_loc.position).ok())
		.and_then(|vp_ray3d| get_flick_vector(*flick_plane, vp_ray3d, f_transl))
	else { return };

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
	let (InputModeState::Select { initial_position: Some(init_pos) }, Some(point_loc)) =
		(r_input_state.mode_state, &pointers.single().location)
	else {
		select_box_boi.map(|x| x.into_inner())
			.map(|(b, _)| cmd.entity(b).despawn());
		return;
	};

	let Aabb2d { min: min @ Vec2 { x: a_x, y: a_y }, max: Vec2 { x: b_x, y: b_y } } =
		aabb_from_points(init_pos, point_loc.position);

	// Guard when the area of the box is zero.
	if (b_x - a_x) * (b_y - a_x) == 0. {
		select_box_boi.map(|x| x.into_inner())
			.map(|(b, _)| cmd.entity(b).despawn());
		return;
	};

	let selection_box_size = UVec2 { x: (b_x - a_x) as u32, y: (b_y - a_y) as u32 };

	if let Some((_, mut node)) = select_box_boi.map(|x| x.into_inner()) {
		node.top    = Val::Px(a_y);
		node.left   = Val::Px(a_x);
		node.width  = Val::Px(selection_box_size.x as f32);
		node.height = Val::Px(selection_box_size.y as f32);
	} else {
		cmd.spawn((
			Node {
				top:    Val::Px(a_y),
				left:   Val::Px(a_x),
				width:  Val::Px(selection_box_size.x as f32),
				height: Val::Px(selection_box_size.y as f32),
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

	let selected_objs : Vec<(Vec3, Entity)> =  visible_objects.iter()
		.filter_map(|(entity, global_transform, aabb, &visibility)|
			(visibility == InheritedVisibility::VISIBLE && contains_obb(
				&sub_frustum,
				aabb,
				&Affine3A::from_mat4(
					cam_transform.compute_matrix().inverse()
						* global_transform.compute_matrix()),
				// FIXME: Should it be true of false ?
				true,
				true,
			)).then(|| (global_transform.translation(), entity)))
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
