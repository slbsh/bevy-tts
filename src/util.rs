use bevy::{math::{Vec2, Vec3}, transform::components::Transform};

pub fn vec2_in_transform(transform: Transform, Vec2 {x, y}: Vec2) -> Vec3 {
    transform.translation + x * transform.right() + y * transform.up()
}
