use bevy::{
	math::{Affine3A, Vec2, Vec3, bounding::Aabb2d},
	transform::components::Transform,
  render::primitives::{Aabb, Frustum}
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

pub fn contains_obb(
    frustum: &Frustum,
    aabb: &Aabb,
    world_from_local: &Affine3A,
    intersect_near: bool,
    intersect_far: bool,
) -> bool {
    let aabb_center_world = world_from_local.transform_point3a(aabb.center).extend(1.0);
    let min_one = f32::MAX;

    for (idx, half_space) in frustum.half_spaces.into_iter().enumerate() {
        if idx == 4 && !intersect_near {
            continue;
        }

        if idx == 5 && !intersect_far {
            continue;
        }

        let p_normal = half_space.normal();
        let relative_radius = aabb.relative_radius(&p_normal, &world_from_local.matrix3);
        if half_space.normal_d().dot(aabb_center_world) + relative_radius < 2. * relative_radius {
            return false;
        }
    }
    true
}
