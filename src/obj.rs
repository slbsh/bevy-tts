use bevy::asset::io::Reader;
use bevy::prelude::*;
use bevy::asset::{AssetLoader, LoadContext, RenderAssetUsages};
use bevy::render::mesh::{Indices, PrimitiveTopology};
use bevy::utils::ConditionalSendFuture;

pub fn objloader(app: &mut App) {
	app.register_asset_loader(ObjLoader)
		.register_asset_loader(MtlLoader);
}

pub struct ObjLoader;

impl AssetLoader for ObjLoader {
	type Error = std::io::Error;
	type Settings = ();
	type Asset = Mesh;

	fn load(&self, reader: &mut dyn Reader, _: &Self::Settings, _: &mut LoadContext)
		-> impl ConditionalSendFuture<Output = Result<Self::Asset, Self::Error>>
	{
		Box::pin(async move {
			let mut buf = Vec::new();
			reader.read_to_end(&mut buf).await?;

			// TODO: materiel? (:
			let (mut model, _) = tobj::load_obj_buf(
				&mut buf.as_slice(),
				&tobj::GPU_LOAD_OPTIONS,
				|_| Err(tobj::LoadError::GenericFailure)).unwrap();

			let mesh = &mut model[0].mesh;

			let mut nmesh = Mesh::new(
				PrimitiveTopology::TriangleList,
				RenderAssetUsages::default());

			// clone 😳
			let to_vec3 = |v: &[f32]| v.chunks_exact(3)
				.map(|v| [v[0], v[1], v[2]]).collect::<Vec<[f32; 3]>>();

			nmesh.insert_indices(Indices::U32(mesh.indices.clone()));
			nmesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, to_vec3(&mesh.positions));

			if !mesh.texcoords.is_empty() {
				nmesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, mesh.texcoords.chunks_exact(2)
					.map(|v| [v[0], 1.0 - v[1]]).collect::<Vec<[f32; 2]>>());
			}

			match mesh.normals.is_empty() {
				true  => nmesh.compute_normals(),
				false => nmesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, to_vec3(&mesh.normals)),
			}

			Ok(nmesh)
		})
	}

	fn extensions(&self) -> &[&str] { &["obj"] }
}


// TODO: currently fairly useless, ideally we can get this 
// to emit a bevy material, its just bit annoyin to do :p
#[derive(Asset, TypePath)]
pub struct Mtl(Vec<tobj::Material>);

pub struct MtlLoader;

impl AssetLoader for MtlLoader {
	type Error = tobj::LoadError;
	type Settings = ();
	type Asset = Mtl;

	fn load(&self, reader: &mut dyn Reader, _: &Self::Settings, _: &mut LoadContext)
		-> impl ConditionalSendFuture<Output = Result<Self::Asset, Self::Error>> 
	{
		Box::pin(async move {
			let mut buf = Vec::new();
			// FIXME: this loses err data, mayhap make an enum
			reader.read_to_end(&mut buf).await
				.map_err(|_| tobj::LoadError::ReadError)?;
			// TODO: map for unhandled keys??
			let (materials, _map) = tobj::load_mtl_buf(&mut buf.as_slice())?;
			Ok(Mtl(materials))
		})
	}

	fn extensions(&self) -> &[&str] { &["mtl"] }
}

// shenanigans for multi mesh obj files
// TODO: decide if to keep
	// fn load(&self, reader: &mut dyn bevy::asset::io::Reader, _: &Self::Settings, _: &mut LoadContext) 
	// 	-> impl ConditionalSendFuture<Output = Result<Self::Asset, Self::Error>> {
	// 	Box::pin(async move {
	// 		let mut buf = Vec::new();
	// 		reader.read_to_end(&mut buf).await?;
	//
	// 		let (model, material) = tobj::load_obj_buf(&mut buf.as_slice(), &tobj::GPU_LOAD_OPTIONS, |_| todo!("wtf is this arg")).unwrap();
	//
	// 		let mut mesh = Mesh::new(
	// 			PrimitiveTopology::TriangleList,
	// 			RenderAssetUsages::default());
	//
	// 		mesh.insert_indices(Indices::U32(meshes.iter().fold(
	// 			(Vec::with_capacity(meshes.iter().map(|m| m.indices.len()).sum()), 0),
	// 			|(mut data, off), mesh| {
	// 				data.extend(mesh.indices.iter().map(|i| i + off));
	// 				(data, off + (mesh.positions.len() / 3) as u32)
	// 			}).0));
	//
	// 		mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, meshes.iter().fold(
	// 			Vec::with_capacity(meshes.iter().map(|m| m.positions.len() / 3).sum()),
	// 			|mut data, mesh| {
	// 				data.append(&mut mesh.positions.chunks_exact(3).map(|v| [v[0], v[1], v[2]]).collect());
	// 				data
	// 			}));
	//
	// 		if meshes.iter().all(|m| !m.texcoords.is_empty()) {
	// 			mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, meshes.iter().fold(
	// 				Vec::with_capacity(meshes.iter().map(|m| m.texcoords.len() / 2).sum()),
	// 				|mut data, mesh| {
	// 					data.append(&mut mesh.texcoords.chunks_exact(2).map(|v| [v[0], 1.0 - v[1]]).collect::<Vec<[f32; 2]>>());
	// 					data
	// 				}));
	// 		}
	//
	// 		if meshes.iter().all(|m| !m.normals.is_empty()) {
	// 			mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, meshes.iter().fold(
	// 				Vec::with_capacity(meshes.iter().map(|m| m.normals.len() / 3).sum()),
	// 				|mut data, mesh| {
	// 					data.append(&mut mesh.normals.chunks_exact(3).map(|v| [v[0], v[1], v[2]]).collect());
	// 					data
	// 				}));
	// 		}
	// 		else {
	// 			mesh.compute_normals();
	// 		}
	//
	// 		Ok(mesh)
	// 	})
	// }
