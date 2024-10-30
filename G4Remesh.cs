using g4;
using Rhino.Geometry;
using System;

namespace G4Library
{
    public class VoxelAndRemeshing
    {
        #region voxels

        private static DMesh3 voxel_surface_old(Mesh rhMesh, int numcells = 64)
        {
            BoundingBox bbox = rhMesh.GetBoundingBox(false);
            DMesh3 mesh = SDFandMeshing.ConvertToDMesh(rhMesh);
            DMeshAABBTree3 spatial = new DMeshAABBTree3(mesh, autoBuild: true);

            AxisAlignedBox3d bounds = mesh.CachedBounds;

            double cellsize = bounds.MaxDim / numcells;
            MeshSignedDistanceGrid levelSet = new MeshSignedDistanceGrid(mesh, cellsize);
            levelSet.UseParallel = true;
            levelSet.Compute();

            Bitmap3 bmp = new Bitmap3(levelSet.Dimensions);
            foreach (Vector3i idx in bmp.Indices())
            {
                float f = levelSet[idx.x, idx.y, idx.z];
                bmp.Set(idx, (f < 0) ? true : false);
            }

            VoxelSurfaceGenerator voxGen = new VoxelSurfaceGenerator();
            voxGen.Voxels = bmp;
            voxGen.ColorSourceF = (idx) =>
            {
                return new Colorf((float)idx.x, (float)idx.y, (float)idx.z) * (1.0f / numcells);
            };
            voxGen.Generate();
            DMesh3 voxMesh = voxGen.Meshes[0];
            MeshNormals.QuickCompute(voxMesh);
            return voxMesh;
        }

        public static DMesh3 VoxelizeMesh(Mesh rhMesh, int numcells)
        {
            // Get the bounding box of the input Rhino mesh
            BoundingBox bbox = rhMesh.GetBoundingBox(false);
            DMesh3 mesh = SDFandMeshing.ConvertToDMesh(rhMesh);

            // Compute dimensions of the bounding box
            double width = bbox.Max.X - bbox.Min.X;
            double height = bbox.Max.Y - bbox.Min.Y;
            double depth = bbox.Max.Z - bbox.Min.Z;

            // Compute the cell size based on the bounding box's diagonal length
            double cellsize = bbox.Diagonal.Length / numcells;

            // Set up the MeshSignedDistanceGrid with the calculated cell size
            MeshSignedDistanceGrid levelSet = new MeshSignedDistanceGrid(mesh, cellsize);

            levelSet.UseParallel = true;

            // Compute the level set
            levelSet.Compute();

            // Create a bitmap to store voxelized data based on the SDF
            Bitmap3 bmp = new Bitmap3(levelSet.Dimensions);
            foreach (Vector3i idx in bmp.Indices())
            {
                float f = levelSet[idx.x, idx.y, idx.z];
                bmp.Set(idx, f < 0); // Set voxel true for inside, false otherwise
            }

            // Generate a voxel surface from the bitmap
            VoxelSurfaceGenerator voxGen = new VoxelSurfaceGenerator
            {
                Voxels = bmp,
            };
            voxGen.Generate();

            DMesh3 voxMesh = voxGen.Meshes[0];

            // Fix position and scale based on the bounding box
            AxisAlignedBox3d voxelBounds = voxMesh.GetBounds();
            double scaleX = width / (voxelBounds.Max.x - voxelBounds.Min.x);
            double scaleY = height / (voxelBounds.Max.y - voxelBounds.Min.y);
            double scaleZ = depth / (voxelBounds.Max.z - voxelBounds.Min.z);

            // Scale each vertex of the voxel mesh
            for (int i = 0; i < voxMesh.VertexCount; i++)
            {
                g4.Vector3d vertex = voxMesh.GetVertex(i); // Get the vertex using its index
                g4.Vector3d scaledVertex = new g4.Vector3d(vertex.x * scaleX, vertex.y * scaleY, vertex.z * scaleZ);
                // Set the vertex back to the mesh
                voxMesh.SetVertex(i, scaledVertex); // Use the index to set the vertex
            }

            // Translate the voxel mesh to align with the original bounding box
            g4.Vector3d offset = new g4.Vector3d(
                bbox.Min.X - (voxelBounds.Min.x * scaleX),
                bbox.Min.Y - (voxelBounds.Min.y * scaleY),
                bbox.Min.Z - (voxelBounds.Min.z * scaleZ)
            );

            for (int i = 0; i < voxMesh.VertexCount; i++)
            {
                g4.Vector3d vertex = voxMesh.GetVertex(i); // Get the vertex using its index
                g4.Vector3d translatedVertex = new g4.Vector3d(vertex.x + offset.x, vertex.y + offset.y, vertex.z + offset.z);
                // Set the vertex back to the mesh
                voxMesh.SetVertex(i, translatedVertex); // Use the index to set the vertex
            }

            return voxMesh;
        }

        private static DMesh3 marching_cubes(Mesh rhMesh, int resolution)
        {
            // Convert Rhino mesh to g3 DMesh3
            DMesh3 mesh = SDFandMeshing.ConvertToDMesh(rhMesh);

            // Set up parameters for MarchingCubes
            MarchingCubes c = new MarchingCubes
            {
                ParallelCompute = true,
                CubeSize = mesh.CachedBounds.MaxDim / resolution, // 128 Adjust CubeSize for resolution
                Bounds = mesh.CachedBounds
            };
            c.Bounds.Expand(c.CubeSize * 2); // Expand slightly to capture boundary

            // Execute Marching Cubes
            LocalProfiler profiler = new LocalProfiler();
            profiler.Start("Generate");
            c.Generate();
            profiler.Stop("Generate");

            // Output triangle count and timing
            Console.WriteLine($"Tris: {c.Mesh.TriangleCount}, Times: {profiler.AllTimes()}");

            // Reduce mesh resolution for optimization
            Reducer r = new Reducer(c.Mesh);
            r.ReduceToEdgeLength(c.CubeSize * 0.25);
            Console.WriteLine($"After reduce: {c.Mesh.TriangleCount}");

            // Compute mesh normals
            MeshNormals.QuickCompute(c.Mesh);
            return c.Mesh;
        }


        private static DMesh3 marching_cubes_levelset(Mesh rhMesh, int numcells = 64)
        {

            DMesh3 mesh = SDFandMeshing.ConvertToDMesh(rhMesh);

            AxisAlignedBox3d bounds = mesh.CachedBounds;
            double cellsize = bounds.MaxDim / numcells;

            MeshSignedDistanceGrid levelSet = new MeshSignedDistanceGrid(mesh, cellsize);
            levelSet.ExactBandWidth = 3;
            //levelSet.InsideMode = MeshSignedDistanceGrid.InsideModes.CrossingCount;
            levelSet.UseParallel = true;
            levelSet.ComputeMode = MeshSignedDistanceGrid.ComputeModes.NarrowBandOnly;
            levelSet.Compute();

            var iso = new DenseGridTrilinearImplicit(levelSet.Grid, levelSet.GridOrigin, levelSet.CellSize);

            MarchingCubes c = new MarchingCubes();
            c.Implicit = iso;
            c.Bounds = mesh.CachedBounds;
            c.Bounds.Expand(c.Bounds.MaxDim * 0.1);
            c.CubeSize = c.Bounds.MaxDim / numcells;//128
            //c.CubeSize = levelSet.CellSize;

            c.Generate();

            return c.Mesh;
        }

        private static DMesh3 marching_cubes_basic(Mesh rhMesh, int numcells = 64)
        {
            // Convert Rhino mesh to g3 DMesh3
            DMesh3 mesh = SDFandMeshing.ConvertToDMesh(rhMesh);
            MeshTransforms.Translate(mesh, -mesh.CachedBounds.Center);

            // Set up Signed Distance Grid (SDF) for the mesh
            double meshCellsize = mesh.CachedBounds.MaxDim / numcells;
            MeshSignedDistanceGrid levelSet = new MeshSignedDistanceGrid(mesh, meshCellsize)
            {
                ExactBandWidth = 3,
                UseParallel = true,
                ComputeMode = MeshSignedDistanceGrid.ComputeModes.NarrowBandOnly
            };
            levelSet.Compute();

            // Set up implicit field from the SDF
            var meshIso = new DenseGridTrilinearImplicit(levelSet.Grid, levelSet.GridOrigin, levelSet.CellSize);

            // Define the bounds and CubeSize for Marching Cubes
            AxisAlignedBox3d bounds = meshIso.Bounds();
            MarchingCubes c = new MarchingCubes
            {
                RootMode = MarchingCubes.RootfindingModes.LerpSteps,
                RootModeSteps = 5,
                Implicit = meshIso,
                Bounds = bounds,
                CubeSize = bounds.MaxDim / numcells
            };
            c.Bounds.Expand(3 * c.CubeSize);

            // Generate the mesh
            c.Generate();

            // Compute normals for the generated mesh
            MeshNormals.QuickCompute(c.Mesh);
            return c.Mesh;
        }

        #endregion 

        /*/
        #region remeshing

        public static Mesh RemeshSmoothing(Mesh rmesh)
        {
            DMesh3 mesh = SDFandMeshing.ConvertToDMesh(rmesh);

            Remesher r = new Remesher(mesh);
            r.EnableFlips = r.EnableSplits = r.EnableCollapses = false;
            r.EnableSmoothing = true;
            r.SmoothSpeedT = 0.5f;
            r.SmoothType = Remesher.SmoothTypes.MeanValue;

            for (int k = 0; k < 100; ++k)
            {
                r.BasicRemeshPass();
                mesh.CheckValidity();
            }

            return SDFandMeshing.ConvertToRhinoMesh(mesh);
        }

        private static bool WriteDebugMeshes = true;

        public static Mesh BasicRemesh(Mesh rmesh, float fResScale = 1.0f)
        {
            DMesh3 mesh = SDFandMeshing.ConvertToDMesh(rmesh);
            MeshUtil.ScaleMesh(mesh, Frame3f.Identity, new g4.Vector3f(1, 2, 1));
            mesh.CheckValidity();

            Remesher r = new Remesher(mesh);
            r.EnableFlips = r.EnableSplits = r.EnableCollapses = true;
            r.MinEdgeLength = 0.1f * fResScale;
            r.MaxEdgeLength = 0.2f * fResScale;
            r.EnableSmoothing = true;
            r.SmoothSpeedT = 0.1f;

            r.EnableFlips = r.EnableSmoothing = false;
            r.MinEdgeLength = 0.05f * fResScale;
            for (int k = 0; k < 10; ++k)
            {
                r.BasicRemeshPass();
                mesh.CheckValidity();
            }

            r.MinEdgeLength = 0.1f * fResScale;
            r.MaxEdgeLength = 0.2f * fResScale;
            r.EnableFlips = r.EnableCollapses = r.EnableSmoothing = true;

            for (int k = 0; k < 10; ++k)
            {
                r.BasicRemeshPass();
                mesh.CheckValidity();
            }

            return SDFandMeshing.ConvertToRhinoMesh(mesh);
        }


        public static Mesh BasicClosedRemesh(Mesh rmesh)
        {
            DMesh3 mesh = SDFandMeshing.ConvertToDMesh(rmesh);
            MeshUtil.ScaleMesh(mesh, Frame3f.Identity, new g4.Vector3f(1, 2, 1));

            mesh.CheckValidity();

            Remesher r = new Remesher(mesh);
            r.EnableFlips = r.EnableSplits = r.EnableCollapses = true;
            r.MinEdgeLength = 0.1f;
            r.MaxEdgeLength = 0.2f;
            r.EnableSmoothing = true;
            r.SmoothSpeedT = 0.1f;

            r.EnableFlips = r.EnableSmoothing = false;
            r.MinEdgeLength = 0.05f;
            for (int k = 0; k < 10; ++k)
            {
                r.BasicRemeshPass();
                mesh.CheckValidity();
            }

            r.MinEdgeLength = 0.1f;
            r.MaxEdgeLength = 0.2f;
            r.EnableFlips = r.EnableCollapses = r.EnableSmoothing = true;

            for (int k = 0; k < 10; ++k)
            {
                r.BasicRemeshPass();
                mesh.CheckValidity();
            }

            r.EnableSplits = r.EnableCollapses = false;

            for (int k = 0; k < 10; ++k)
            {
                r.BasicRemeshPass();
                mesh.CheckValidity();
            }

            return SDFandMeshing.ConvertToRhinoMesh(mesh);
        }


        public static Mesh RemeshSmoothing(Mesh rmesh)
        {
            DMesh3 mesh = SDFandMeshing.ConvertToDMesh(rmesh);

            Remesher r = new Remesher(mesh);
            r.EnableFlips = r.EnableSplits = r.EnableCollapses = false;
            r.EnableSmoothing = true;
            r.SmoothSpeedT = 0.5f;
            r.SmoothType = Remesher.SmoothTypes.MeanValue;

            for (int k = 0; k < 100; ++k)
            {
                r.BasicRemeshPass();
                mesh.CheckValidity();
            }

           return SDFandMeshing.ConvertToRhinoMesh(mesh);
        }


        public static Mesh RemeshConstraintsFixedverts(Mesh rmesh, int Slices = 128)
        {
            DMesh3 mesh = SDFandMeshing.ConvertToDMesh(rmesh);
            MeshUtil.ScaleMesh(mesh, Frame3f.Identity, new g4.Vector3f(1, 2, 1));
            mesh.CheckValidity();
            AxisAlignedBox3d bounds = mesh.CachedBounds;

            // construct mesh projection target
            DMesh3 meshCopy = new DMesh3(mesh);
            meshCopy.CheckValidity();
            DMeshAABBTree3 tree = new DMeshAABBTree3(meshCopy);
            tree.Build();
            MeshProjectionTarget target = new MeshProjectionTarget()
            {
                Mesh = meshCopy,
                Spatial = tree
            };

            // construct constraint set
            MeshConstraints cons = new MeshConstraints();

            //EdgeRefineFlags useFlags = EdgeRefineFlags.NoFlip | EdgeRefineFlags.NoCollapse;
            EdgeRefineFlags useFlags = EdgeRefineFlags.NoFlip;

            foreach (int eid in mesh.EdgeIndices())
            {
                double fAngle = MeshUtil.OpeningAngleD(mesh, eid);
                if (fAngle > 30.0f)
                {
                    cons.SetOrUpdateEdgeConstraint(eid, new EdgeConstraint(useFlags));
                    Index2i ev = mesh.GetEdgeV(eid);
                    int nSetID0 = (mesh.GetVertex(ev[0]).y > bounds.Center.y) ? 1 : 2;
                    int nSetID1 = (mesh.GetVertex(ev[1]).y > bounds.Center.y) ? 1 : 2;
                    cons.SetOrUpdateVertexConstraint(ev[0], new VertexConstraint(true, nSetID0));
                    cons.SetOrUpdateVertexConstraint(ev[1], new VertexConstraint(true, nSetID1));
                }
            }

            Remesher r = new Remesher(mesh);
            r.Precompute();
            r.SetExternalConstraints(cons);
            r.SetProjectionTarget(target);

            var stopwatch = Stopwatch.StartNew();

            //double fResScale = 1.0f;
            double fResScale = 0.5f;
            r.EnableFlips = r.EnableSplits = r.EnableCollapses = true;
            r.MinEdgeLength = 0.1f * fResScale;
            r.MaxEdgeLength = 0.2f * fResScale;
            r.EnableSmoothing = true;
            r.SmoothSpeedT = 0.5f;

            try
            {
                for (int k = 0; k < 20; ++k)
                {
                    r.BasicRemeshPass();
                    mesh.CheckValidity();
                }
            }
            catch
            {
                // ignore
            }

            stopwatch.Stop();
            System.Console.WriteLine("Second Pass Timing: " + stopwatch.Elapsed);

            return SDFandMeshing.ConvertToRhinoMesh(mesh);
        }


        private static Mesh RemeshRegion(Mesh rmesh, int Slices = 16)
        {        
            DMesh3 mesh = SDFandMeshing.ConvertToDMesh(rmesh);
            MeshUtil.ScaleMesh(mesh, Frame3f.Identity, new g4.Vector3f(1, 2, 1));
            mesh.CheckValidity();

            int[] tris = GetTrisOnPositiveSide(mesh, new Frame3f(g4.Vector3f.Zero, g4.Vector3f.AxisY));

            RegionRemesher r = new RegionRemesher(mesh, tris);
            r.Region.SubMesh.CheckValidity(true);

            r.Precompute();
            double fResScale = 0.5f;
            r.EnableFlips = r.EnableSplits = r.EnableCollapses = true;
            r.MinEdgeLength = 0.1f * fResScale;
            r.MaxEdgeLength = 0.2f * fResScale;
            r.EnableSmoothing = true;
            r.SmoothSpeedT = 1.0f;

            for (int k = 0; k < 5; ++k)
            {
                r.BasicRemeshPass();
                mesh.CheckValidity();
            }

            r.BackPropropagate();

            for (int k = 0; k < 5; ++k)
            {
                r.BasicRemeshPass();
                mesh.CheckValidity();
            }

            r.BackPropropagate();

            return SDFandMeshing.ConvertToRhinoMesh(mesh);
        }

        private static int[] GetTrisOnPositiveSide(DMesh3 mesh, Frame3f plane)
        {
            DVector<int> keep_tris = new DVector<int>();

            g4.Vector3d[] tri = new g4.Vector3d[3];
            foreach (int tid in mesh.TriangleIndices())
            {
                mesh.GetTriVertices(tid, ref tri[0], ref tri[1], ref tri[2]);
                bool ok = true;
                for (int j = 0; j < 3; ++j)
                {
                    double d = (tri[j] - plane.Origin).Dot(plane.Z);
                    if (d < 0)
                        ok = false;
                }
                if (ok)
                    keep_tris.Add(tid);
            }

            return keep_tris.GetBuffer();
        }

        #endregion
        /*/
    }
}
