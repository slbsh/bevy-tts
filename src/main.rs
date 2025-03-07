use bevy::log::LogPlugin;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use bevy_inspector_egui::quick::{ResourceInspectorPlugin, WorldInspectorPlugin};

pub mod camera;
pub mod game;
pub mod user_input;

fn main() {
	App::new()
		.add_plugins((
			// Cabbage man needs this LogPlugin to filter gpu errors.
			DefaultPlugins.set(LogPlugin {
				filter: "wgpu_hal=off".into(),
				level: bevy::log::Level::INFO,
				custom_layer: |_app| None,
			}),
			MeshPickingPlugin,
			RapierPickingPlugin,
			RapierPhysicsPlugin::<NoUserData>::default().in_schedule(FixedPostUpdate),
			RapierDebugRenderPlugin::default(),
			WorldInspectorPlugin::default(),
			ResourceInspectorPlugin::<MeshPickingSettings>::default(),
			ResourceInspectorPlugin::<user_input::InputState>::default(),
			user_input::plugin,
			camera::plugin,
			game::plugin,
		))
		.insert_resource(ClearColor(Color::srgb(0.97, 0.97, 1.0)))
		.insert_resource(MeshPickingSettings {
			require_markers: true,
			ray_cast_visibility: RayCastVisibility::Visible,
		})
		.run();
}
