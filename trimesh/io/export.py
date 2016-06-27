import numpy as np
import json

from ..constants import log
from ..util      import tolist_dict, is_string, array_to_encoded

from .stl import export_stl
from .ply import export_ply
#python 3
try:                from cStringIO import StringIO
except ImportError: from io import StringIO

def export_mesh(mesh, file_obj, file_type=None):
    '''
    Export a Trimesh object to a file- like object, or to a filename

    Arguments
    ---------
    file_obj: a filename string or a file-like object
    file_type: str representing file type (eg: 'stl')
    process:   boolean flag, whether to process the mesh on load

    Returns:
    mesh: a single Trimesh object, or a list of Trimesh objects, 
          depending on the file format. 
    
    '''
    if is_string(file_obj):
        file_type = (str(file_obj).split('.')[-1]).lower()
        file_obj  = open(file_obj, 'wb')
    file_type = str(file_type).lower()
    
    log.info('Exporting %d faces as %s', len(mesh.faces), file_type.upper())
    export = _mesh_exporters[file_type](mesh)
    
    if hasattr(file_obj, 'write'):
        file_obj.write(export)
        file_obj.flush()
        file_obj.close()
    else:
        return export

def export_off(mesh):
    faces_stacked = np.column_stack((np.ones(len(mesh.faces))*3, mesh.faces)).astype(np.int64)
    export = 'OFF\n'
    export += str(len(mesh.vertices)) + ' ' + str(len(mesh.faces)) + ' 0\n'
    export += np.array_str(mesh.vertices, precision=9).replace('[', '').replace(']', '').strip()
    export += np.array_str(faces_stacked).replace('[', '').replace(']', '').strip()

    return export

def export_collada(mesh):
    '''
    Export a mesh as a COLLADA file.
    '''
    from ..templates import get_template
    from string import Template

    template_string = get_template('collada.dae.template')
    template = Template(template_string)

    # we bother setting this because np.array2string uses these printoptions 
    np.set_printoptions(threshold=np.inf, precision=5, linewidth=np.inf)
    replacement = dict()
    replacement['VERTEX']   = np.array2string(mesh.vertices.reshape(-1))[1:-1]
    replacement['FACES']    = np.array2string(mesh.faces.reshape(-1))[1:-1]
    replacement['NORMALS']  = np.array2string(mesh.vertex_normals.reshape(-1))[1:-1]
    replacement['VCOUNT']   = str(len(mesh.vertices))
    replacement['VCOUNTX3'] = str(len(mesh.vertices) * 3)
    replacement['FCOUNT']   = str(len(mesh.faces))
    
    export = template.substitute(replacement)
    return export

def export_dict64(mesh):
    return export_dict(mesh, encoding='base64')

def export_dict(mesh, encoding=None):
    def encode(item, dtype=None):
        if encoding is None:
            return item.tolist()
        else:
            return array_to_encoded(item, 
                                    dtype = dtype, 
                                    encoding = encoding)
                                    
    export = {'metadata'     : tolist_dict(mesh.metadata),
              'faces'        : encode(mesh.faces,        np.uint32),
              'face_normals' : encode(mesh.face_normals, np.float32),
              'vertices'     : encode(mesh.vertices,     np.float32)}
    return export
        
def export_json(mesh):
    blob = export_dict(mesh, encoding='base64')
    export = json.dumps(blob)
    return export
    
def export_msgpack(mesh):
    import msgpack
    blob = export_dict(mesh, encoding='binary')
    export = msgpack.dumps(blob)
    return export
                         
_mesh_exporters = {'ply'  : export_ply, 
                   'stl'  : export_stl,
                   'dict' : export_dict,
                   'json' : export_json,
                   'off'  : export_off,
                   'dae'  : export_collada,
                   'dict64'  : export_dict64,
                   'msgpack' : export_msgpack,
                   'collada' : export_collada}
