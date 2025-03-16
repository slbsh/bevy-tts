use std::path::PathBuf;

use bevy::asset::io::Reader;
use bevy::prelude::*;
use bevy::asset::{AssetLoader, LoadContext, RenderAssetUsages};
use bevy::render::mesh::{Indices, PrimitiveTopology};
use bevy::utils::ConditionalSendFuture;

pub fn objloader(app: &mut App) {
	app.register_asset_loader(ObjLoader)
		.register_asset_loader(MtlLoader)
		.init_asset::<Obj>();
}

#[derive(Reflect, Asset)]
pub struct Obj {
	pub name:     String,
	pub mesh:     Mesh,
	pub material: Option<Handle<StandardMaterial>>,
}

struct ObjLoader;
impl AssetLoader for ObjLoader {
	type Error = std::io::Error;
	type Settings = ();
	type Asset = Obj;

	fn load(&self, reader: &mut dyn Reader, _: &Self::Settings, ctx: &mut LoadContext)
		-> impl ConditionalSendFuture<Output = Result<Self::Asset, Self::Error>>
	{
		Box::pin(async move {
			let mut buf = Vec::new();
			reader.read_to_end(&mut buf).await?;

			// ugly hack, but no other way methinks >_<
			static mut MTLPATH: Option<PathBuf> = None;

			let (mut model, _) = tobj::load_obj_buf(
				&mut buf.as_slice(),
				&tobj::GPU_LOAD_OPTIONS,
				|p| { unsafe { MTLPATH = Some(p.to_owned()) }; Err(tobj::LoadError::GenericFailure) }
			).unwrap();

			#[allow(static_mut_refs)]
			let material = unsafe { MTLPATH.as_ref() }
				.map(|p| ctx.load::<StandardMaterial>(p.as_path()));

			let meshname = std::mem::take(&mut model[0].name);
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

			// FIXME: this fucks up the normals (but is also required for em?) :(
			if !mesh.normals.is_empty() && !mesh.texcoords.is_empty() {
				nmesh.generate_tangents();
			}

			Ok(Obj {
				name: meshname,
				mesh: nmesh,
				material,
			})
		})
	}

	fn extensions(&self) -> &[&str] { &["obj"] }
}


struct MtlLoader;
impl AssetLoader for MtlLoader {
	type Error = std::io::Error;
	type Settings = ();
	type Asset = StandardMaterial;

	fn load(&self, reader: &mut dyn Reader, _: &Self::Settings, ctx: &mut LoadContext)
		-> impl ConditionalSendFuture<Output = Result<Self::Asset, Self::Error>> 
	{
		Box::pin(async move {
			let mut buf = Vec::new();
			reader.read_to_end(&mut buf).await?;
			// TODO: unhandled keys? (:
			let (mut material, _) = tobj::load_mtl_buf(&mut buf.as_slice()).unwrap();
			let mat = &mut material[0];

			// TODO: map the rest of these to StandardMaterial
			// /// Ambient color of the material.
			// pub ambient: Option<[Float; 3]>,
			// /// Specular color of the material.
			// pub specular: Option<[Float; 3]>,
			// /// Material shininess attribute. Also called `glossiness`.
			// pub dissolve: Option<Float>,
			// /// Name of the ambient texture file for the material.
			// pub ambient_texture: Option<String>,
			// /// Name of the specular texture file for the material.
			// pub specular_texture: Option<String>,
			// /// Name of the shininess map texture file for the material.
			// pub shininess_texture: Option<String>,
			// /// Name of the alpha/opacity map texture file for the material.
			// /// Referred to as `dissolve` to match the `MTL` file format specification.
			// pub dissolve_texture: Option<String>,
			// /// The illumnination model to use for this material. The different
			// /// illumination models are specified in the [`MTL` spec](http://paulbourke.net/dataformats/mtl/).
			// pub illumination_model: Option<u8>,
			// /// Key value pairs of any unrecognized parameters encountered while parsing the material.
			// pub unknown_param: HashMap<String, String>,

			println!("{:#?}", mat);


			Ok(StandardMaterial {
				//reflectance: mat.shininess.unwrap_or(0.5),

				ior: mat.optical_density.unwrap_or(1.5),
				base_color: mat.diffuse
					.map(|v| Color::srgb(v[0], v[1], v[2]))
					.unwrap_or(Color::WHITE),

				base_color_texture: mat.diffuse_texture.as_ref().map(|p| ctx.load(p)),
				normal_map_texture: mat.normal_texture.as_ref().map(|p| ctx.load(p)),

				..Default::default()
			})
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
