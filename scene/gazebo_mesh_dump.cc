#include <gazebo/common/MeshManager.hh>
#include <gazebo/common/Mesh.hh>
#include <fstream>
#include <iostream>
#include <cstdint>
int main(int argc,char**argv) {
 if(argc!=3) return 2;
 auto m=gazebo::common::MeshManager::Instance()->Load(argv[1]);
 if(!m) return 3;
 std::ofstream out(argv[2],std::ios::binary);
 uint32_t n=m->GetSubMeshCount();out.write((char*)&n,4);
 for(uint32_t k=0;k<n;k++) {
  auto s=m->GetSubMesh(k);
  if(s->GetPrimitiveType()!=gazebo::common::SubMesh::TRIANGLES) return 5;
  uint32_t nv=s->GetVertexCount(),ni=s->GetIndexCount();
  std::cerr<<k<<" "<<s->GetName()<<" vertices="<<nv<<" indices="<<ni<<" min="<<s->Min()<<" max="<<s->Max()<<"\n";
  out.write((char*)&nv,4);out.write((char*)&ni,4);
  for(uint32_t i=0;i<nv;i++){auto v=s->Vertex(i);double a[3]={v.X(),v.Y(),v.Z()};out.write((char*)a,24);}
  for(uint32_t i=0;i<ni;i++){uint32_t a=s->GetIndex(i);out.write((char*)&a,4);}
 }
 return out.good()?0:4;
}
