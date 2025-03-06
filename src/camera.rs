use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;

#[derive(Component)]
#[require(Camera)]
pub struct MainCamera;

pub fn plugin(app: &mut App) {
	app.add_systems(Startup, setup).add_systems(Update, camera_movement);
}

fn setup(mut commands: Commands) {
	commands.spawn((
		Camera3d::default(),
		MainCamera,
		Transform::from_xyz(0.0, 30.0, 0.0),
		RayCastPickable,
	));
}

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
