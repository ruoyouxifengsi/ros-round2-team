from pathlib import Path
import sys
src=Path(sys.argv[1]);dest=Path(sys.argv[2]);adapter=Path(sys.argv[3])
original=(src/'gazebo/physics/ode/ODEPhysics.cc').read_text()
start=original.index('void ODEPhysics::Collide(')
end=original.index('\n//////////////////////////////////////////////////',start)
headers=original[:original.index('GZ_REGISTER_PHYSICS_ENGINE')]
dest.write_text(headers+'\n'+adapter.read_text()+'\n'+original[start:end])
