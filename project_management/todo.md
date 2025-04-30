~~Placement Height~~
~~Lockable objects (Locks selection)~~
	This is accomplished by removing RaycastPickable from the thing
~~Anchored objects (Disables physics simulation).~~
	This is accomplished by setting `RigidBody::Static`. Don't naively remove RigidBody, because Rapier breaks.
Mouse drag tool should align object with global Y-axis
Dragging multiple objects should select random points inside an Aabb to lerp towards (and random orientation).
	Physics disabled.
Separate Gizmo Transformation Tools
~~Flick tool~~
Joints
Uploadable 3d Objects
Returnable Objects (If fall off table, come back table)
Save Scenes
Flip Table (with revertable state)

(X) Multi-selection
