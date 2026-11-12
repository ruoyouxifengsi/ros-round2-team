// Experimental Gazebo 11.15.1 adapter. Original ODEPhysics update structure:
// Copyright Open Source Robotics Foundation, Apache-2.0.
// No installed library or physics parameter is overwritten.
#include <gazebo/physics/ode/ODEPhysics.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/SurfaceParams.hh>
#include <gazebo/physics/PhysicsFactory.hh>
#include <gazebo/common/Plugin.hh>
#include "ODEPhysicsPrivate.hh"
#include <gazebo/ode/collision_trimesh.h>
#include <dlfcn.h>
#include <omp.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace {
using Contacts = std::array<dContactGeom, MAX_COLLIDE_RETURNS>;
using CollideFn = int (*)(dGeomID,dGeomID,int,dContactGeom*,int);
CollideFn realCollide() {
  static auto fn = reinterpret_cast<CollideFn>(dlsym(dlopen("libgazebo_ode.so.11",RTLD_NOW),"dCollide"));
  if (!fn) std::abort();
  return fn;
}
struct Work {
  dGeomID a=nullptr,b=nullptr,ca=nullptr,cb=nullptr;
  Contacts contacts{};
  int count=0;
};
thread_local Work *cached=nullptr;
unsigned long steps=0,pairs=0,hits=0,verified=0,mismatches=0;
double parallelSeconds=0,updateSeconds=0;
using Clock=std::chrono::steady_clock;
bool supported(dGeomID g) {
  int type=dGeomGetClass(g);
  if (type==dPlaneClass || type==dBoxClass || type==dSphereClass ||
      type==dCapsuleClass || type==dCylinderClass) return true;
  return type==dTriMeshClass && !dGeomTriMeshGetCallback(g) &&
    !dGeomTriMeshGetArrayCallback(g) && !dGeomTriMeshGetTriMergeCallback(g);
}
dGeomID clone(dGeomID g) {
  if(dGeomGetClass(g)==dPlaneClass) {
    dVector4 p;dGeomPlaneGetParams(g,p);
    return dCreatePlane(nullptr,p[0],p[1],p[2],p[3]);
  }
  dGeomID copy=nullptr;
  switch(dGeomGetClass(g)) {
    case dBoxClass: {
      dVector3 sides;dGeomBoxGetLengths(g,sides);
      copy=dCreateBox(nullptr,sides[0],sides[1],sides[2]);break;
    }
    case dSphereClass: copy=dCreateSphere(nullptr,dGeomSphereGetRadius(g));break;
    case dCapsuleClass: {
      dReal radius,length;dGeomCapsuleGetParams(g,&radius,&length);
      copy=dCreateCapsule(nullptr,radius,length);break;
    }
    case dCylinderClass: {
      dReal radius,length;dGeomCylinderGetParams(g,&radius,&length);
      copy=dCreateCylinder(nullptr,radius,length);break;
    }
    default: copy=dCreateTriMesh(nullptr,dGeomTriMeshGetData(g),nullptr,nullptr,nullptr);
  }
  const dReal *p=dGeomGetPosition(g);
  dGeomSetPosition(copy,p[0],p[1],p[2]);
  dGeomSetRotation(copy,dGeomGetRotation(g));
  return copy;
}
bool equal(const Work &w,const Contacts &reference,int count) {
  if(w.count!=count)return false;
  for(int i=0;i<count;++i) {
    const auto &a=w.contacts[i];const auto &b=reference[i];
    if(a.side1!=b.side1 || a.side2!=b.side2 || !std::isfinite(a.depth) ||
       !std::isfinite(b.depth) || std::abs(a.depth-b.depth)>1e-10)return false;
    for(int j=0;j<3;++j)
      if(!std::isfinite(a.pos[j]) || !std::isfinite(b.pos[j]) ||
         !std::isfinite(a.normal[j]) || !std::isfinite(b.normal[j]) ||
         std::abs(a.pos[j]-b.pos[j])>1e-10 || std::abs(a.normal[j]-b.normal[j])>1e-10)return false;
  }
  return true;
}
void stats() {
  std::fprintf(stderr,"[parallel-collision] steps=%lu pairs=%lu cache_hits=%lu verified=%lu mismatches=%lu parallel_seconds=%.6f update_seconds=%.6f\n",steps,pairs,hits,verified,mismatches,parallelSeconds,updateSeconds);
}
}

extern "C" int dCollide(dGeomID a,dGeomID b,int flags,dContactGeom *out,int stride) {
  if(cached && cached->a==a && cached->b==b && flags==MAX_COLLIDE_RETURNS) {
    for(int i=0;i<cached->count;++i)
      std::memcpy(reinterpret_cast<char*>(out)+i*stride,&cached->contacts[i],sizeof(dContactGeom));
    ++hits;
    return cached->count;
  }
  return realCollide()(a,b,flags,out,stride);
}

void gazebo::physics::ODEPhysics::UpdateCollision() {
  const auto updateStart=Clock::now();
  static const int threads=[] {
    if(!dCheckConfiguration("ODE_EXT_mt_collisions")) {
      std::fprintf(stderr,"[parallel-collision] ODE lacks thread-local collision support\n");std::abort();
    }
    const char *value=std::getenv("ROUND2_COLLISION_THREADS");
    int n=value?std::atoi(value):4;
    std::atexit(stats);
    std::fprintf(stderr,"[parallel-collision] experimental adapter active; workers=%d\n",n);
    return std::max(1,std::min(20,n));
  }();
  static const bool verify=std::getenv("ROUND2_COLLISION_VERIFY")!=nullptr;
  static const int verifyStride=[] {
    const char *value=std::getenv("ROUND2_COLLISION_VERIFY_STRIDE");
    return value?std::max(1,std::atoi(value)):1;
  }();
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);
  dJointGroupEmpty(this->dataPtr->contactGroup);
  this->dataPtr->collidersCount=0;
  this->dataPtr->trimeshCollidersCount=0;
  this->dataPtr->jointFeedbackIndex=0;
  this->contactManager->ResetCount();
  dSpaceCollide(this->dataPtr->spaceId,this,CollisionCallback);
  const int ordinary=this->dataPtr->collidersCount;
  const int size=ordinary+this->dataPtr->trimeshCollidersCount;
  auto pairAt=[&](int i) {return i<ordinary?this->dataPtr->colliders[i]:this->dataPtr->trimeshColliders[i-ordinary];};
  std::vector<Work> work(size);
  for(int i=0;i<size;++i) {
    auto &w=work[i];auto pair=pairAt(i);
    auto s1=pair.first->GetSurface();auto s2=pair.second->GetSurface();
    if(!(s1->collideBitmask & s2->collideBitmask))continue;
    if((s1->collideWithoutContact || s2->collideWithoutContact) &&
       !(s1->collideWithoutContactBitmask & s2->collideWithoutContactBitmask))continue;
    w.a=pair.first->GetCollisionId();w.b=pair.second->GetCollisionId();
    if(supported(w.a) && supported(w.b)) {w.ca=clone(w.a);w.cb=clone(w.b);}
  }
  auto fn=realCollide();
  const auto parallelStart=Clock::now();
  #pragma omp parallel for num_threads(threads) schedule(dynamic,1)
  for(int i=0;i<size;++i) {
    auto &w=work[i];if(!w.ca)continue;
    if(!dAllocateODEDataForThread(dAllocateMaskAll))std::abort();
    w.count=fn(w.ca,w.cb,MAX_COLLIDE_RETURNS,w.contacts.data(),sizeof(dContactGeom));
    for(int j=0;j<w.count;++j) {
      w.contacts[j].g1=w.a;w.contacts[j].g2=w.b;
    }
  }
  parallelSeconds+=std::chrono::duration<double>(Clock::now()-parallelStart).count();
  for(int i=0;i<size;++i) {
    auto &w=work[i];auto pair=pairAt(i);
    if(w.ca) {
      ++pairs;
      if(verify && steps%verifyStride==0) {
        Contacts reference{};
        int count=fn(w.a,w.b,MAX_COLLIDE_RETURNS,reference.data(),sizeof(dContactGeom));
        ++verified;
        if(!equal(w,reference,count)) {
          ++mismatches;
          std::fprintf(stderr,"[parallel-collision] contact mismatch; aborting experimental simulation\n");
          stats();std::abort();
        }
      }
      cached=&w;
    }
    this->Collide(pair.first,pair.second,this->dataPtr->contactCollisions);
    cached=nullptr;
    if(w.ca) {dGeomDestroy(w.ca);dGeomDestroy(w.cb);}
  }
  updateSeconds+=std::chrono::duration<double>(Clock::now()-updateStart).count();
  if(++steps%1000==0)stats();
}

namespace gazebo {
class ExperimentalODE : public physics::ODEPhysics {
 public: explicit ExperimentalODE(physics::WorldPtr world):physics::ODEPhysics(world) {}
 public: void UpdateCollision() override { physics::ODEPhysics::UpdateCollision(); }
};
physics::PhysicsEnginePtr createExperimental(physics::WorldPtr world) {
  return physics::PhysicsEnginePtr(new ExperimentalODE(world));
}
class ParallelCollisionPlugin : public SystemPlugin {
 public: void Load(int,char **) override {
   physics::PhysicsFactory::RegisterPhysicsEngine("round2_parallel_ode",createExperimental);
 }
};
GZ_REGISTER_SYSTEM_PLUGIN(ParallelCollisionPlugin)
}
