use bevy::{
	math::{Vec2, Vec3, bounding::Aabb2d},
	transform::components::Transform,
};

pub fn vec2_in_transform(transform: Transform, Vec2 { x, y }: Vec2) -> Vec3 {
	transform.translation + x * transform.right() + y * transform.up()
}

/// Gets an Aabb from 2 points. This method is useful for cases where
/// `a` and `b` are neither the top-left / bottom-right corners of the Aabb.
pub fn aabb_from_points(
	Vec2 { x: a_x, y: a_y }: Vec2,
	Vec2 { x: b_x, y: b_y }: Vec2,
) -> Aabb2d {
	Aabb2d {
		min: Vec2 { x: a_x.min(b_x), y: a_y.min(b_y) },
		max: Vec2 { x: a_x.max(b_x), y: a_y.max(b_y) },
	}
}
