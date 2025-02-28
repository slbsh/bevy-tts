use bevy::{input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll}, prelude::*};
use bevy_rapier3d::prelude::*;

fn main() {
	App::new()
		.add_plugins((
			DefaultPlugins,
			RapierPickingPlugin,
			RapierPhysicsPlugin::<NoUserData>::default(),
			RapierDebugRenderPlugin::default(),
		))
		.insert_resource(ClearColor(Color::srgb(0.97, 0.97, 1.0)))
		.add_systems(Startup, setup)
		.add_systems(Update, camera_movement)
		.run();
}

fn camera_movement(
	mut query: Query<&mut Transform, With<Camera>>,
	keys:   Res<ButtonInput<KeyCode>>,
	mouse:  Res<ButtonInput<MouseButton>>,
	motion: Res<AccumulatedMouseMotion>,
	scroll: Res<AccumulatedMouseScroll>,
	time:   Res<Time>,
) {
	let mut transform = query.single_mut();
	let mut dir = Vec3::ZERO;
	let speed = 40.0 * time.delta_secs();

	keys.get_pressed().for_each(|k|
		match k {
			KeyCode::KeyW  => dir += *transform.forward(),
			KeyCode::KeyS  => dir -= *transform.forward(),
			KeyCode::KeyA  => dir -= *transform.right(),
			KeyCode::KeyD  => dir += *transform.right(),
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
	mut commands:  Commands,
	mut meshes:    ResMut<Assets<Mesh>>,
	mut materials: ResMut<Assets<StandardMaterial>>,
) {
	// Ground
	commands.spawn((
		Mesh3d(meshes.add(Cuboid::new(200.0, 0.2, 200.0))),
		Collider::cuboid(100.0, 0.1, 100.0),
		MeshMaterial3d(materials.add(Color::srgb(0.7, 0.7, 0.8))),
		RigidBody::Fixed,
		ColliderDebugColor(Hsla::BLACK),
	));

	// cubes
	let num = 8;
	let rad = 1.0;

	let shift = rad * 2.0 + rad;
	let centerx = shift * (num / 2) as f32;
	let centery = shift / 2.0;
	let centerz = shift * (num / 2) as f32;

	let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;
	let mut color = 0;
	let colors = [
		Hsla::hsl(220.0, 1.0, 0.3),
		Hsla::hsl(180.0, 1.0, 0.3),
		Hsla::hsl(260.0, 1.0, 0.7),
	];


	let cube_mesh = meshes.add(Cuboid::new(rad*2.0, rad*2.0, rad*2.0));

	for j in 0usize..20 {
		for i in 0..num {
			for k in 0usize..num {
				let x = i as f32 * shift - centerx + offset;
				let y = j as f32 * shift + centery + 3.0;
				let z = k as f32 * shift - centerz + offset;
				color += 1;

				commands
					.spawn((Transform::from_rotation(Quat::from_rotation_x(0.2)), Visibility::default()))
					.with_children(|child| {
						child.spawn((
							Mesh3d(cube_mesh.clone()),
							MeshMaterial3d(materials.add(Color::srgb(0.2, 0.7, 0.9))),
							Collider::cuboid(rad, rad, rad),
							Transform::from_xyz(x, y, z),
							RigidBody::Dynamic,
							ColliderDebugColor(colors[color % 3]),
							RapierPickable,
						))
						.observe(cursor_drag);
					});
			}
		}

		offset -= 0.05 * rad * (num as f32 - 1.0);
	}

	// Directional light
	commands.spawn((
		DirectionalLight {
			illuminance: 5000.0,
			shadows_enabled: true,
			..default()
		},
		Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
	));

	// camera
	commands.spawn((Camera3d::default(), Transform::from_xyz(0.0, 30.0, 0.0)));
}

fn cursor_drag(
	drag: Trigger<Pointer<Drag>>, 
	mut transforms: Query<&mut Transform>,
) {
	if let Ok(mut transform) = transforms.get_mut(drag.entity()) {
		// move the cube around
		transform.translation.x -= drag.delta.x;
		transform.translation.y -= drag.delta.y;
	}
}
