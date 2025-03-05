use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::log::LogPlugin;
use bevy::picking::mesh_picking::ray_cast::RayMeshHit;
use bevy::prelude::*;
use bevy::render::mesh::MeshAabb;
use bevy::render::primitives::Aabb;
use bevy_rapier3d::prelude::*;

pub const MOUSE_RAY_TOI: f32 = 1000.;

fn main() {
	App::new()
		.add_plugins((
			// Cabbage man needs this LogPlugin to filter gpu errors.
			DefaultPlugins.set(LogPlugin {
				filter:       "wgpu_hal=off".into(),
				level:        bevy::log::Level::INFO,
				custom_layer: |_app| None,
			}),
			MeshPickingPlugin,
			RapierPickingPlugin,
			RapierPhysicsPlugin::<NoUserData>::default(),
			RapierDebugRenderPlugin::default(),
		))
		.insert_resource(ClearColor(Color::srgb(0.97, 0.97, 1.0)))
		.insert_resource(MeshPickingSettings {
			require_markers:     true,
			ray_cast_visibility: RayCastVisibility::Visible,
		})
		.add_systems(Startup, setup)
		.add_systems(Update, camera_movement)
		.run();
}

#[derive(Component)]
#[require(Camera)]
pub struct MainCamera;

/// Marker Component for Entities that can have objects place ontop of themm (used in mouse raycas)
#[derive(Component)]
pub struct PlacablePlatform;

fn camera_movement(
	mut query: Query<&mut Transform, With<MainCamera>>,
	keys: Res<ButtonInput<KeyCode>>,
	mouse: Res<ButtonInput<MouseButton>>,
	motion: Res<AccumulatedMouseMotion>,
	scroll: Res<AccumulatedMouseScroll>,
	time: Res<Time>,
) {
	let mut transform = query.single_mut();
	let mut dir = Vec3::ZERO;
	let speed = 40.0 * time.delta_secs();

	keys.get_pressed().for_each(|k| match k {
		KeyCode::KeyW => dir += *transform.forward(),
		KeyCode::KeyS => dir -= *transform.forward(),
		KeyCode::KeyA => dir -= *transform.right(),
		KeyCode::KeyD => dir += *transform.right(),
		_ => (),
	});

	if mouse.pressed(MouseButton::Right) {
		transform.rotate(Quat::from_rotation_y(-motion.delta.x * 0.003));
		transform.rotate_local_x(-motion.delta.y * 0.003);
	}

	if scroll.delta.y != 0.0 {
		let dir = transform.forward();
		transform.translation += dir * scroll.delta.y * speed * 8.0;
	}

	transform.translation += dir * speed;
}

fn setup(
	mut commands: Commands,
	mut meshes: ResMut<Assets<Mesh>>,
	mut materials: ResMut<Assets<StandardMaterial>>,
) {
	// Ground
	commands.spawn((
		Mesh3d(meshes.add(Cuboid::new(200.0, 0.2, 200.0))),
		Collider::cuboid(100.0, 0.1, 100.0),
		MeshMaterial3d(materials.add(Color::srgb(0.7, 0.7, 0.8))),
		RigidBody::Fixed,
		ColliderDebugColor(Hsla::BLACK),
		PlacablePlatform,
	));

	// Table
	commands.spawn((
		Mesh3d(meshes.add(Cuboid::new(20.0, 0.2, 20.0))),
		Transform::from_translation(Vec3::new(25.0, 10.0, 25.0)),
		Collider::cuboid(10.0, 0.1, 10.0),
		MeshMaterial3d(materials.add(Color::srgb(0.7, 0.3, 0.3))),
		RigidBody::Fixed,
		ColliderDebugColor(Hsla::BLACK),
		PlacablePlatform,
	));

	// cubes
	let num = 3;
	let rad = 1.0;

	let shift = rad * 2.0 + rad;
	let centerx = shift * (num / 2) as f32;
	let centery = shift / 2.0;
	let centerz = shift * (num / 2) as f32;

	let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;
	let mut color = 0;
	let colors =
		[Hsla::hsl(220.0, 1.0, 0.3), Hsla::hsl(180.0, 1.0, 0.3), Hsla::hsl(260.0, 1.0, 0.7)];

	let cube_mesh = meshes.add(Cuboid::new(rad * 2.0, rad * 2.0, rad * 2.0));

	for i in 0..num {
		for k in 0usize..num {
			let x = i as f32 * shift - centerx + offset;
			let y = shift + centery + 3.0;
			let z = k as f32 * shift - centerz + offset;
			color += 1;

			commands
				.spawn((
					Transform {
						translation: Vec3 { x, y, z },
						rotation: Quat::from_rotation_x(0.2),
						..default()
					},
					Visibility::default(),
					Mesh3d(cube_mesh.clone()),
					MeshMaterial3d(materials.add(Color::srgb(0.2, 0.7, 0.9))),
					RayCastPickable,
					Collider::cuboid(rad, rad, rad),
					RigidBody::Dynamic,
					// ColliderDebugColor(colors[color % 3]),
					// RapierPickable,
				))
				.observe(cursor_drag);
		}

		offset -= 0.05 * rad * (num as f32 - 1.0);
	}

	// Directional light
	commands.spawn((
		DirectionalLight { illuminance: 5000.0, shadows_enabled: true, ..default() },
		Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
	));

	// camera
	commands.spawn((
		Camera3d::default(),
		MainCamera,
		Transform::from_xyz(0.0, 30.0, 0.0),
		RayCastPickable,
	));
}

fn cursor_drag(
	drag: Trigger<Pointer<Drag>>,
	mut mesh_cast_param: MeshRayCast,
	mut transforms: Query<&mut Transform>,
	q_mesh: Query<&Mesh3d>,
	q_cam: Query<&Camera, With<MainCamera>>,
	q_gt: Query<&GlobalTransform, With<MainCamera>>,
	q_has_placable: Query<(), With<PlacablePlatform>>,
	r_mesh: Res<Assets<Mesh>>,
) {
	let cam = q_cam.single();
	let cam_global_t = q_gt.single();
	let Ok(ray3d) = cam.viewport_to_world(cam_global_t, drag.pointer_location.position)
	else {
		return;
	};

	// TODO: Use `ColisionGroups`
	let Some((_, RayMeshHit { point, normal, .. })) = mesh_cast_param
		.cast_ray(ray3d, &RayCastSettings {
			visibility: RayCastVisibility::VisibleInView,
			filter: &|e| q_has_placable.contains(e),
			..default()
		})
		.first()
	else {
		return;
	};

	let &Transform { rotation, scale, .. } = transforms.get(drag.entity()).unwrap();

	let mut transform = transforms.get_mut(drag.entity()).unwrap();
	let Mesh3d(mesh_h) = q_mesh.get(drag.entity()).unwrap();
	let mesh = r_mesh.get(mesh_h).unwrap();

	// TODO: In general, likely needs to use Aabb center.
	let Aabb { half_extents, .. } = mesh.compute_aabb().unwrap();

	*transform = Transform {
		translation: *point + half_extents.y * normal,
		rotation: Quat::from_axis_angle(*normal, rotation.to_axis_angle().1),
		scale,
	};
}
