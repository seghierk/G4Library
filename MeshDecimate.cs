using MeshDecimator;
using MeshDecimator.Math;

namespace G4Library
{
    public class MeshDecimate
    {
        public static Rhino.Geometry.Mesh DecimateMesh(Rhino.Geometry.Mesh inputMesh, double quality, bool qualityType)
        {
            if (inputMesh == null) return null;

            quality = MathHelper.Clamp01((float)quality);

            // Convert Rhino mesh to MeshDecimator format
            var vertices = new Vector3d[inputMesh.Vertices.Count];
            for (int i = 0; i < inputMesh.Vertices.Count; i++)
            {
                vertices[i] = new Vector3(inputMesh.Vertices[i].X, inputMesh.Vertices[i].Y, inputMesh.Vertices[i].Z);
            }

            var faces = new int[inputMesh.Faces.Count * 3];
            for (int i = 0, j = 0; i < inputMesh.Faces.Count; i++, j += 3)
            {
                faces[j] = inputMesh.Faces[i].A;
                faces[j + 1] = inputMesh.Faces[i].B;
                faces[j + 2] = inputMesh.Faces[i].C;
            }

            var sourceMesh = new Mesh(vertices, new[] { faces });

            // Apply decimation
            var targetTriangleCount = (int)(sourceMesh.TriangleCount * quality);

            var algorithm = MeshDecimation.CreateAlgorithm(Algorithm.Default);

            Mesh decimatedMesh;
            if (qualityType == false)
                decimatedMesh = MeshDecimation.DecimateMesh(algorithm, sourceMesh, targetTriangleCount);
            else
                decimatedMesh = MeshDecimation.DecimateMeshLossless(algorithm, sourceMesh);

            // Convert back to Rhino mesh
            var outputMesh = new Rhino.Geometry.Mesh();
            foreach (var vertex in decimatedMesh.Vertices)
            {
                outputMesh.Vertices.Add(vertex.x, vertex.y, vertex.z);
            }

            var decimatedIndices = decimatedMesh.GetSubMeshIndices();
            for (int i = 0; i < decimatedIndices.Length; i++)
            {
                for (int j = 0; j < decimatedIndices[i].Length; j += 3)
                {
                    outputMesh.Faces.AddFace(
                        decimatedIndices[i][j],
                        decimatedIndices[i][j + 1],
                        decimatedIndices[i][j + 2]
                    );
                }
            }

            outputMesh.Normals.ComputeNormals();
            outputMesh.Compact();

            // Output the decimated mesh
            return outputMesh;
        }
    }
}
