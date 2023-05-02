# Mesh Reconstruction

### This project demonstrates decoding ply file either in ascii or binary form and methods for surface reconstruction, including Poisson and Ball-Pivoting Algorithm.

1. Original ply: [Mouse.ply](https://github.com/Chen-Si-An/MeshReconstruction/blob/main/setup/Mouse.ply)

![Image](https://github.com/Chen-Si-An/MeshReconstruction/blob/main/Mouse_ply.bmp)

2. Surface reconstruction through Poisson: [Mouse_Poisson.stl](https://github.com/Chen-Si-An/MeshReconstruction/blob/main/setup/Mouse_Poisson.stl)

![Image](https://github.com/Chen-Si-An/MeshReconstruction/blob/main/Mouse_mesh_Poisson.bmp)

3. Surface reconstruction through BPA: [Mouse_BPA_OMP.stl](https://github.com/Chen-Si-An/MeshReconstruction/blob/main/setup/Mouse_BPA_OMP.stl)

![Image](https://github.com/Chen-Si-An/MeshReconstruction/blob/main/Mouse_mesh_BPA.bmp)

**Particulary, within the method of BPA, the process of finding seed can be accelerated by GPU, and to utilize GPU computing, the steps are listed as following:**
  1. Install CUDA Toolkits. Note that to directly compile this project, you should install the version of 11.6.
  2. Switch the configuration of VS to "Debug_GPU" or "Release_GPU".

**Note: BPA is mainly based on <https://github.com/rodschulz/BPA>, with some revisions.**
