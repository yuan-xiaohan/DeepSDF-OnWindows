import numpy as np
from mesh_to_sdf.mesh_to_sdf import get_surface_point_cloud, scale_to_unit_sphere, scale_to_unit_cube,\
    sample_sdf_near_surface, sample_inside_surface, ComputeNormalizationParameters
import trimesh
import skimage, skimage.measure
import os
import pyrender


def writeSDFToNPZ(filename, points, sdf):
    pos_xyz = points[np.where(sdf > 0)]
    pos_sdf = sdf[np.where(sdf > 0)].reshape(-1, 1)
    pos = np.concatenate((pos_xyz, pos_sdf), axis=1)

    neg_xyz = points[np.where(sdf < 0)]
    neg_sdf = sdf[np.where(sdf < 0)].reshape(-1, 1)
    neg = np.concatenate((neg_xyz, neg_sdf), axis=1)

    np.savez(filename, pos=pos, neg=neg)

    """load npz file"""
    # sdf_load = np.load(filename)
    # print(sdf_load)


def PreprocessMesh(input_name, output_name, test_sampling=True):
    mesh = trimesh.load(input_name)
    # mesh = scale_to_unit_sphere(mesh)
    # # print("Scanning...")
    # # Returns an intermediate data structure containing a surface point cloud, scans and a kd-tree of the point cloud.
    # cloud = get_surface_point_cloud(mesh, surface_point_method='scan', scan_count=20, scan_resolution=400)
    #
    # # print("Voxelizing...")
    # voxels = cloud.get_voxels(128, use_depth_buffer=True)
    #
    # # print("Creating a mesh using Marching Cubes...")
    # vertices, faces, normals, _ = skimage.measure.marching_cubes_lewiner(voxels, level=0)
    # mesh = trimesh.Trimesh(vertices=vertices, faces=faces, vertex_normals=normals)

    mesh = scale_to_unit_sphere(mesh)
    sampling_type = "sphere"
    # Sample some points uniformly and some points near the shape surface and calculate SDFs for these points. This follows the procedure proposed in the DeepSDF paper. The mesh is first transformed to fit inside the unit sphere.
    points, sdf = sample_sdf_near_surface(mesh, sampling_type=sampling_type, test_sampling=test_sampling)
    colors = np.zeros(points.shape)
    colors[sdf < 0, 2] = 1
    colors[sdf > 0, 0] = 1
    # cloud = pyrender.Mesh.from_points(points, colors=colors)
    # scene = pyrender.Scene()
    # scene.add(cloud)
    # viewer = pyrender.Viewer(scene, use_raymond_lighting=True, point_size=2)

    writeSDFToNPZ(output_name, points, sdf)


def SampleVisibleMeshSurface(input_name, output_name, normalization_param_filename, num_sample=30000):
    mesh = trimesh.load(input_name)

    # Sample points on the mesh surface and export ply file
    # surface_points = sample_inside_surface(mesh, surface_point_method='scan', scan_count=20, scan_resolution=400)
    surface_points = mesh.sample(num_sample)
    mesh_out = trimesh.Trimesh(vertices=surface_points)
    mesh_out.export(output_name)

    # Get Normalization Parameters and export npz file
    offset, scale = ComputeNormalizationParameters(surface_points)
    np.savez(normalization_param_filename, offset=offset, scale=scale)


def check(result_path, gt_path):
    result_mesh = trimesh.load(result_path)
    gt_mesh = trimesh.load(gt_path)
    gt_mesh = scale_to_unit_sphere(gt_mesh)
    result_mesh.visual.vertex_colors = [0, 255, 0]
    gt_mesh.visual.vertex_colors = [255, 0, 0]
    gt_mesh.show()
    result_mesh.show()

    # result = pyrender.Mesh.from_trimesh(result_mesh)
    # gt = pyrender.Mesh.from_trimesh(gt_mesh)
    # scene = pyrender.Scene()
    # scene.add(result)
    # scene.add(gt)
    # pyrender.Viewer(scene)


if __name__ == "__main__":
    # name_list = os.listdir(r"\\SEUVCL-DATA-01\Medical\[PRO]SDF\obj_pca\2")
    # for name in name_list:
    #     name = os.path.splitext(name)[0]
    #
    #     result_path = os.path.join(r"D:\XiaohanYuan\SDF\examples\cardiac\Reconstructions\1070\Meshes\cardiac\2",
    #                                name + ".ply")
    #     gt_path = os.path.join(r"\\SEUVCL-DATA-01\Medical\[PRO]SDF\obj\2", name + ".obj")
    #     check(result_path, gt_path)

        # gt_mesh = trimesh.load(gt_path)
        # gt_mesh = scale_to_unit_sphere(gt_mesh)
        # gt_output = r"D:\XiaohanYuan\SDF\examples\cardiac\Reconstructions\gt"
        # gt_mesh.export(os.path.join(gt_output, name+".ply"))

        # check(result_path, gt_path)

    # PreprocessMesh(r'D:\XiaohanYuan\SDF\mesh_to_sdf\unitest\LVV.obj', r"D:\XiaohanYuan\SDF\mesh_to_sdf\unitest\heart.npz")
    SampleVisibleMeshSurface(r'D:\XiaohanYuan\SDF\mesh_to_sdf\unitest\LVV.obj',
                             r"D:\XiaohanYuan\SDF\mesh_to_sdf\unitest\heart.ply",
                             r"D:\XiaohanYuan\SDF\mesh_to_sdf\unitest\heart_nor.obj")

    #
    # name_list = os.listdir(r"\\SEUVCL-DATA-01\Medical\[PRO]SDF\obj_pca\1")
    # for name in name_list:
    #     name = os.path.splitext(name)[0]
    #     print('"'+name+'"'+",")

    # mesh = trimesh.load(r'D:\XiaohanYuan\SDF\mesh_to_sdf\example\model_normalized.obj')
    #
    # ## function1:
    # mesh = scale_to_unit_sphere(mesh)
    #
    # print("Scanning...")
    # # Returns an intermediate data structure containing a surface point cloud, scans and a kd-tree of the point cloud.
    # cloud = get_surface_point_cloud(mesh, surface_point_method='scan', scan_count=20, scan_resolution=400)
    # cloud.show()
    #
    # os.makedirs("test", exist_ok=True)
    # for i, scan in enumerate(cloud.scans):
    #     scan.save(r"D:\XiaohanYuan\SDF\mesh_to_sdf\test\scan_{:d}.png".format(i))
    #
    # ## function2:
    # print("Voxelizing...")
    # voxels = cloud.get_voxels(128, use_depth_buffer=True)
    #
    # print("Creating a mesh using Marching Cubes...")
    # vertices, faces, normals, _ = skimage.measure.marching_cubes(voxels, level=0)
    # mesh = trimesh.Trimesh(vertices=vertices, faces=faces, vertex_normals=normals)
    # mesh.show()
    #
    # ## function3:
    # # scale 对齐类型，还不确定是否会对采样有影响，暂时认为和采样没有关系
    # mesh = scale_to_unit_sphere(mesh)
    # # mesh = scale_to_unit_cube(mesh)
    #
    # sampling_type = "sphere"
    # # Sample some points uniformly and some points near the shape surface and calculate SDFs for these points. This follows the procedure proposed in the DeepSDF paper. The mesh is first transformed to fit inside the unit sphere.
    # points, sdf = sample_sdf_near_surface(mesh, number_of_points=250000, sampling_type=sampling_type)
    # colors = np.zeros(points.shape)
    # colors[sdf < 0, 2] = 1
    # colors[sdf > 0, 0] = 1
    # cloud = pyrender.Mesh.from_points(points, colors=colors)
    # scene = pyrender.Scene()
    # scene.add(cloud)
    # viewer = pyrender.Viewer(scene, use_raymond_lighting=True, point_size=2)
    #
    # ## function4:
    # """
    #     in official DeepSDF code, the final message saved as follows:
    #     if (save_npz == std::string::npos)
    #         writeSDFToNPY(xyz, sdf, npyFileName);
    #     else {
    #         writeSDFToNPZ(xyz, sdf, npyFileName, true);
    #     }
    #
    #     so, if we obtain sampling points as param:xyz, and sdf values as param:sdf, we can get same result like C++
    #     now, based on these two params obtained by this python repo, we save them into .npz format
    # """
    # filename = "chair.npz"
    # writeSDFToNPZ(filename, points, sdf)

