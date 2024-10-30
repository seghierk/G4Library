using g4;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;

namespace G4Library
{
    public class SDFandMeshing
    {
        public static double Clamp(double value, double min, double max)
        {
            return (value < min) ? min : (value > max) ? max : value;
        }

        /*/
        public static (MeshSignedDistanceGrid, DMesh3) Test(string objFile)
        {
            // Read the input mesh
            DMesh3 mesh = StandardMeshReader.ReadMesh(objFile);

            // Set up the Signed Distance Grid (SDF)
            int num_cells = 128;
            double cell_size = mesh.CachedBounds.MaxDim / num_cells;
            MeshSignedDistanceGrid sdf = new MeshSignedDistanceGrid(mesh, cell_size);
            sdf.Compute();

            // Create an implicit function for the Marching Cubes
            var iso = new DenseGridTrilinearImplicit(sdf.Grid, sdf.GridOrigin, sdf.CellSize);

            // Set up Marching Cubes
            g3.MarchingCubes c = new g3.MarchingCubes
            {
                Implicit = iso,
                Bounds = mesh.CachedBounds,
                CubeSize = mesh.CachedBounds.MaxDim / 128
            };
            c.Bounds.Expand(3 * c.CubeSize);

            // Generate the output mesh
            c.Generate();
            DMesh3 outputMesh = c.Mesh;

            // Return a tuple of SDF and output mesh
            return (sdf, outputMesh);
        }
        /*/
        // Helper function to convert Rhino Mesh to g3 DMesh3

        private static Line[] GetBBoxLines(DMesh3 dmesh)
        {
            var bbox = new BoundingBox();
            var box = dmesh.GetBounds();
            bbox.Min = new Point3d(box.Min.x, box.Min.y, box.Min.z);
            bbox.Max = new Point3d(box.Max.x, box.Max.y, box.Max.z);

            return bbox.GetEdges();
        }

        public static GH_GeometryGroup GetBBox(DMesh3 dmesh)
        {
            var lines = GetBBoxLines(dmesh);
            GH_GeometryGroup grp = new GH_GeometryGroup();
            var goos = lines.Select(m => GH_Convert.ToGeometricGoo(m));
            grp.Objects.AddRange(goos);
            return grp;
        }

        private static DMesh3 DmeshReduce(DMesh3 dmesh, int count)
        {
            // Reduce mesh resolution for optimization
            Reducer r = new Reducer(dmesh);
            r.ReduceToVertexCount(count);
            //r.DoReduce();

            return dmesh;
        }

        public static DMesh3 ConvertToDMesh(Rhino.Geometry.Mesh rhinoMesh)
        {
            int vertexCount = rhinoMesh.Vertices.Count;
            int faceCount = rhinoMesh.Faces.Count;

            // Prepare vertex list as Vector3f
            List<g4.Vector3f> vertexList = new List<g4.Vector3f>(vertexCount);
            for (int i = 0; i < vertexCount; i++)
            {
                var vertex = rhinoMesh.Vertices[i];
                vertexList.Add(new g4.Vector3f(vertex.X, vertex.Y, vertex.Z));
            }

            // Prepare triangle list as Index3i
            List<Index3i> triangleList = new List<Index3i>(faceCount);
            for (int i = 0; i < faceCount; i++)
            {
                var face = rhinoMesh.Faces[i];
                if (face.IsQuad)
                {
                    // Split quad into two triangles
                    triangleList.Add(new Index3i(face.A, face.B, face.C));
                    triangleList.Add(new Index3i(face.A, face.C, face.D));
                }
                else
                {
                    triangleList.Add(new Index3i(face.A, face.B, face.C));
                }
            }

            DMesh3 result = DMesh3Builder.Build<g4.Vector3f, Index3i, g4.Vector3f>(vertexList, triangleList, null, null);

            return result;
        }


        public static (MeshSignedDistanceGrid, DMesh3) GenerateSDFAndMesh(Rhino.Geometry.Mesh rhinoMesh, int num_cells, double cubeSizeRatio)
        {
            // Convert Rhino mesh to g3 DMesh3 more efficiently
            DMesh3 inputMesh = ConvertToDMesh(rhinoMesh);

            // Set up the Signed Distance Grid (SDF)
            double cell_size = inputMesh.CachedBounds.MaxDim / num_cells;
            MeshSignedDistanceGrid sdf = new MeshSignedDistanceGrid(inputMesh, cell_size);
            sdf.Compute();

            // Create an implicit function for the Marching Cubes
            var iso = new DenseGridTrilinearImplicit(sdf.Grid, sdf.GridOrigin, sdf.CellSize);

            // Set up and run Marching Cubes with optimized CubeSize
            g4.MarchingCubes c = new g4.MarchingCubes
            {
                Implicit = iso,
                Bounds = inputMesh.CachedBounds,
                CubeSize = cell_size * cubeSizeRatio,
                ParallelCompute = true
            };
            c.Bounds.Expand(2 * c.CubeSize);

            // Generate the output mesh
            c.Generate();
            DMesh3 outputMesh = c.Mesh;

            return (sdf, outputMesh);
        }


        // Convert g3Sharp DMesh3 to Rhino.Geometry.Mesh
        public static Rhino.Geometry.Mesh ConvertToRhinoMesh(DMesh3 dMesh)
        {
            DMesh3 dmesh = new DMesh3(dMesh);
            Rhino.Geometry.Mesh rhinoMesh = new Rhino.Geometry.Mesh();

            // Add vertices
            foreach (g4.Vector3d vertex in dmesh.Vertices())
            {
                rhinoMesh.Vertices.Add((float)vertex.x, (float)vertex.y, (float)vertex.z);
            }

            // Add faces
            foreach (Index3i face in dmesh.Triangles())
            {
                rhinoMesh.Faces.AddFace(face.a, face.b, face.c);
            }

            // Recalculate normals and compact mesh
            //rhinoMesh.Normals.ComputeNormals();

            rhinoMesh.Vertices.CombineIdentical(true, true);
            rhinoMesh.Faces.CullDegenerateFaces();
            rhinoMesh.UnifyNormals();
            rhinoMesh.RebuildNormals();
            rhinoMesh.Compact();

            return rhinoMesh;
        }

        #region booleans

        public static BoundedImplicitFunction3d ConvertToImplicitFunction(MeshSignedDistanceGrid levelSet)
        {
            // Create a DenseGridTrilinearImplicit from the levelSet grid, origin, and cell size
            return new DenseGridTrilinearImplicit(levelSet.Grid, levelSet.GridOrigin, levelSet.CellSize);
        }

        public static DMesh3 ConvertImplicitToMesh(BoundedImplicitFunction3d implicitFunc, int numCells, double cubeSizeRatio)
        {
            // Calculate base CubeSize using the bounding box and number of cells
            double baseCubeSize = implicitFunc.Bounds().MaxDim / numCells;

            // Set up Marching Cubes with adjusted CubeSize based on cubeSizeRatio
            MarchingCubes c = new MarchingCubes
            {
                Implicit = implicitFunc,
                RootMode = MarchingCubes.RootfindingModes.LerpSteps,
                RootModeSteps = 5,
                Bounds = implicitFunc.Bounds(),
                CubeSize = baseCubeSize * cubeSizeRatio  // Apply cubeSizeRatio
            };

            // Optional buffer to expand bounds around the mesh
            c.Bounds.Expand(3 * c.CubeSize);

            // Generate the mesh using Marching Cubes
            c.Generate();
            MeshNormals.QuickCompute(c.Mesh);

            return c.Mesh;
        }

        public static Func<BoundedImplicitFunction3d, int, double, DMesh3> generateMeshF = (root, numCells, cubeSizeRatio) =>
        {
            // Calculate base CubeSize using the bounding box and number of cells
            double baseCubeSize = root.Bounds().MaxDim / numCells;

            // Set up Marching Cubes with adjusted CubeSize based on cubeSizeRatio
            MarchingCubes c = new MarchingCubes
            {
                Implicit = root,
                RootMode = MarchingCubes.RootfindingModes.LerpSteps,
                RootModeSteps = 5,
                Bounds = root.Bounds(),
                CubeSize = baseCubeSize * cubeSizeRatio  // Apply cubeSizeRatio
            };

            // Optional buffer to expand bounds around the mesh
            c.Bounds.Expand(3 * c.CubeSize);

            // Generate the mesh using Marching Cubes
            c.Generate();
            MeshNormals.QuickCompute(c.Mesh);

            // Return the generated mesh
            return c.Mesh;
        };

        /*/
        // Generic function for creating and saving a mesh from an implicit function
        public static void GenerateMesh(BoundedImplicitFunction3d implicitFunc, int numCells, string path)
        {
            MarchingCubes c = new MarchingCubes
            {
                Implicit = implicitFunc,
                RootMode = MarchingCubes.RootfindingModes.LerpSteps,
                RootModeSteps = 5,
                Bounds = implicitFunc.Bounds(),
                CubeSize = implicitFunc.Bounds().MaxDim / numCells
            };
            c.Bounds.Expand(3 * c.CubeSize);
            c.Generate();
            MeshNormals.QuickCompute(c.Mesh);
            StandardMeshWriter.WriteMesh(path, c.Mesh, WriteOptions.Defaults);
        }

        static Action<BoundedImplicitFunction3d, int, string> generateMeshF = (root, numcells, path) =>
        {
            MarchingCubes c = new MarchingCubes();
            c.Implicit = root;
            c.RootMode = MarchingCubes.RootfindingModes.LerpSteps;      // interpolation for better accuracy
            c.RootModeSteps = 5;                                        // steps for convergence
            c.Bounds = root.Bounds();
            c.CubeSize = c.Bounds.MaxDim / numcells;                    // cell resolution
            c.Bounds.Expand(3 * c.CubeSize);                            // add boundary buffer
            c.Generate();
            MeshNormals.QuickCompute(c.Mesh);                           // compute normals for shading
            StandardMeshWriter.WriteMesh(path, c.Mesh, WriteOptions.Defaults); // write output mesh to file
        };
        /*/

        public static Func<DMesh3, int, double, BoundedImplicitFunction3d> meshToImplicitF = (meshIn, numcells, max_offset) =>
        {
            double meshCellsize = meshIn.CachedBounds.MaxDim / numcells;         // cell size based on resolution
            MeshSignedDistanceGrid levelSet = new MeshSignedDistanceGrid(meshIn, meshCellsize);
            levelSet.ExactBandWidth = (int)(max_offset / meshCellsize) + 1;      // narrow band width
            levelSet.Compute();
            return new DenseGridTrilinearImplicit(levelSet.Grid, levelSet.GridOrigin, levelSet.CellSize); // output implicit
        };

        public static Func<DMesh3, int, BoundedImplicitFunction3d> meshToBlendImplicitF = (meshIn, numcells) =>
        {
            double meshCellsize = meshIn.CachedBounds.MaxDim / numcells;
            MeshSignedDistanceGrid levelSet = new MeshSignedDistanceGrid(meshIn, meshCellsize);
            levelSet.ExpandBounds = meshIn.CachedBounds.Diagonal * 0.25;        // expansion for blending
            levelSet.ComputeMode = MeshSignedDistanceGrid.ComputeModes.FullGrid; // compute entire grid
            levelSet.Compute();
            return new DenseGridTrilinearImplicit(levelSet.Grid, levelSet.GridOrigin, levelSet.CellSize);
        };

        private static (DMesh3 latticeMesh, DMesh3 clippedLatticeMesh, DMesh3 finalShellMesh) GenerateLatticeWithShell(DMesh3 mesh, double lattice_radius, double lattice_spacing, double shell_thickness, int mesh_resolution, double cubeSizeRatio)
        {
            // Setup
            var shellMeshImplicit = meshToImplicitF(mesh, 128, shell_thickness);
            double max_dim = mesh.CachedBounds.MaxDim;
            AxisAlignedBox3d bounds = new AxisAlignedBox3d(mesh.CachedBounds.Center, max_dim / 2);
            bounds.Expand(2 * lattice_spacing);
            AxisAlignedBox2d element = new AxisAlignedBox2d(lattice_spacing);
            AxisAlignedBox2d bounds_xy = new AxisAlignedBox2d(bounds.Min.xy, bounds.Max.xy);
            AxisAlignedBox2d bounds_xz = new AxisAlignedBox2d(bounds.Min.xz, bounds.Max.xz);
            AxisAlignedBox2d bounds_yz = new AxisAlignedBox2d(bounds.Min.yz, bounds.Max.yz);

            // Create lattice structure
            List<BoundedImplicitFunction3d> tiling = new List<BoundedImplicitFunction3d>();
            foreach (g4.Vector2d uv in TilingUtil.BoundedRegularTiling2(element, bounds_xy, 0))
            {
                Segment3d seg = new Segment3d(new g4.Vector3d(uv.x, uv.y, bounds.Min.z), new g4.Vector3d(uv.x, uv.y, bounds.Max.z));
                tiling.Add(new ImplicitLine3d { Segment = seg, Radius = lattice_radius });
            }
            foreach (g4.Vector2d uv in TilingUtil.BoundedRegularTiling2(element, bounds_xz, 0))
            {
                Segment3d seg = new Segment3d(new g4.Vector3d(uv.x, bounds.Min.y, uv.y), new g4.Vector3d(uv.x, bounds.Max.y, uv.y));
                tiling.Add(new ImplicitLine3d { Segment = seg, Radius = lattice_radius });
            }
            foreach (g4.Vector2d uv in TilingUtil.BoundedRegularTiling2(element, bounds_yz, 0))
            {
                Segment3d seg = new Segment3d(new g4.Vector3d(bounds.Min.x, uv.x, uv.y), new g4.Vector3d(bounds.Max.x, uv.x, uv.y));
                tiling.Add(new ImplicitLine3d { Segment = seg, Radius = lattice_radius });
            }
            ImplicitNaryUnion3d lattice = new ImplicitNaryUnion3d { Children = tiling };
            DMesh3 latticeMesh = generateMeshF(lattice, 128, cubeSizeRatio);

            // Clip the lattice
            ImplicitIntersection3d latticeClipped = new ImplicitIntersection3d { A = lattice, B = shellMeshImplicit };
            DMesh3 clippedLatticeMesh = generateMeshF(latticeClipped, mesh_resolution, cubeSizeRatio);

            // Create shell
            var shell = new ImplicitDifference3d
            {
                A = shellMeshImplicit,
                B = new ImplicitOffset3d { A = shellMeshImplicit, Offset = -shell_thickness }
            };

            // Union shell and clipped lattice
            DMesh3 finalShellMesh = generateMeshF(new ImplicitUnion3d { A = latticeClipped, B = shell }, mesh_resolution, cubeSizeRatio);

            // Return the generated meshes
            return (latticeMesh, clippedLatticeMesh, finalShellMesh);
        }


        #endregion

    }
}
