use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::user_input::PlacablePlatform;

pub fn plugin(app: &mut App) {
	app.add_systems(Startup, setup)
		.add_systems(Update,  thing);
}

fn thing(
	mut commands: Commands,
	mut e: EventReader<AssetEvent<crate::obj::Obj>>,
	objs: Res<Assets<crate::obj::Obj>>,
	mut meshes: ResMut<Assets<Mesh>>,
	mut materials: ResMut<Assets<StandardMaterial>>,
) {
	e.read().for_each(|e|
		if let AssetEvent::LoadedWithDependencies { id, .. } = e {
			let obj = objs.get(*id).unwrap();

			commands.spawn((
				Mesh3d(meshes.add(obj.mesh.clone())),
				// TODO: generate collider from mesh 🥺👉👈
				Visibility::default(),
				RayCastPickable,
				PlacablePlatform,
				RigidBody::Dynamic,
				if let Some(material) = &obj.material {
					MeshMaterial3d(material.clone())
				} else {
					MeshMaterial3d(materials.add(Color::srgb(0.2, 0.7, 0.9)))
				},
			));
		});
}

fn setup(
	mut commands: Commands,
	mut meshes: ResMut<Assets<Mesh>>,
	mut materials: ResMut<Assets<StandardMaterial>>,
	asset_server: Res<AssetServer>,
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

	// Directional light
	commands.spawn((
		DirectionalLight { illuminance: 5000.0, shadows_enabled: true, ..default() },
		Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
	));

	// let num = 3;
	// let rad = 1.0;
	//
	// let shift = rad * 2.0 + rad;
	// let centerx = shift * (num / 2) as f32;
	// let centery = shift / 2.0;
	// let centerz = shift * (num / 2) as f32;
	//
	// let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;
	//
	// let cube_mesh = meshes.add(Sphere::new(rad));
	//
	// for i in 0..num {
	// 	for k in 0usize..num {
	// 		let x = i as f32 * shift - centerx + offset;
	// 		let y = shift + centery + 3.0;
	// 		let z = k as f32 * shift - centerz + offset;
	//
	// 		commands.spawn((
	// 			Transform { translation: Vec3 { x, y, z }, ..default() },
	// 			Visibility::default(),
	// 			Mesh3d(cube_mesh.clone()),
	// 			MeshMaterial3d(materials.add(Color::srgb(0.2, 0.7, 0.9))),
	// 			RayCastPickable,
	// 			PlacablePlatform,
	// 			Collider::ball(rad),
	// 			RigidBody::Dynamic,
	// 			// RapierPickable,
	// 		));
	// 	}
	//
	// 	offset -= 0.05 * rad * (num as f32 - 1.0);
	// }

	// lol :L
	std::mem::forget(asset_server.load::<crate::obj::Obj>("d6.obj"));
}
