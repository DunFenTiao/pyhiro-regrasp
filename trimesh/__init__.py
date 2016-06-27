'''
The following code is a copy of
https://github.com/mikedh/trimesh.git

The original author is Mike dh (sidd's lab)

author: weiwei
date: 20160627

trimesh.py
========
Python library for loading triangular meshes and doing simple operations on them. Included loaders are binary/ASCII STL and Wavefront (OBJ), included exporters are binary STL or COLLADA. If Assimp/pyassimp are available, meshes can be loaded using the assimp loaders.

Using
-----
    >>> import trimesh
    >>> m = trimesh.load_mesh('models/ballA.off')
    >>> m.show()

'''

from .version import __version__
from .base    import Trimesh

from .util    import unitize
from .points  import transform_points
from .io.load import load_mesh, load_path, load, available_formats

from . import transformations
from . import primitives