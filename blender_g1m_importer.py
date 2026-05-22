# Imports each G1M file in a folder as its own collection.
# Run from Blender text editor with Alt+P.

# Created By: Thunderlol
# https://github.com/the-real-thunderlol/BLENDER-G1M-IMPORTER
# GitHub: the-real-thunderlol
# LICENSE: GPL-3.0

###################
# Script is based on:

# ============================================================
# https://github.com/Joschuka/fmt_g1m
# GitHub: Joschuka
# LICENSE: GPL-3.0
# ============================================================
# https://github.com/eArmada8/gust_stuff
# GitHub: eArmada8
# LICENSE: GPL-3.0
# ============================================================

###################



import bpy, os, io, struct, re, glob, math

# ============================================================
# SETTINGS
# ============================================================
G1M_FOLDER = r"C:\{path/to/folder}"
# ============================================================


# ================================================================
# QUATERNION MATH
# ================================================================

def quat_mul(q1, q2):
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return [
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ]

def quat_norm(q):
    mag = (q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2) ** 0.5
    if mag < 1e-10:
        return [1.0, 0.0, 0.0, 0.0]
    return [v / mag for v in q]

def quat_rotate(q, v):
    qv   = [0.0, v[0], v[1], v[2]]
    qc   = [q[0], -q[1], -q[2], -q[3]]
    temp = quat_mul(q, qv)
    r    = quat_mul(temp, qc)
    return [r[1], r[2], r[3]]

def vec_add(a, b):
    return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]


# ================================================================
# G1M PARSING
# ================================================================

def read_pascal_string(f):
    sz = int.from_bytes(f.read(1), byteorder="little")
    return f.read(sz)

def binary_oid_to_dict(oid_file):
    bones = {}
    with open(oid_file, 'rb') as f:
        f_length = f.seek(0, io.SEEK_END)
        f.seek(-1, 2)
        if f.read(1) == b'\xff':
            f.seek(0)
            s = read_pascal_string(f).decode("ASCII")
            if s == "HeaderCharaOid":
                while f.tell() < f_length:
                    s = read_pascal_string(f).decode("ASCII")
                    if ',' in s:
                        bones[int(s.split(',')[0])] = s.split(',')[1]
            else:
                f.seek(0)
                i = 0
                while f.tell() < f_length:
                    bones[i] = read_pascal_string(f).decode("ASCII")
                    i += 1
    return bones

def parseG1MS(chunk, e):
    s = {}
    with io.BytesIO(chunk) as f:
        s["magic"] = f.read(4).decode("utf-8")
        s["version"], s["size"] = struct.unpack(e+"II", f.read(8))
        jointDataOffset, _ = struct.unpack(e+"II", f.read(8))
        s["jointCount"], s["jointIndicesCount"], s["layer"] = struct.unpack(e+"HHH", f.read(6))
        f.seek(2, 1)
        boneIDList = []
        boneToBoneID = {}
        for i in range(s["jointIndicesCount"]):
            id, = struct.unpack(e+"H", f.read(2))
            boneIDList.append(id)
            if id != 0xFFFF:
                boneToBoneID[id] = i
        s["boneIDList"] = boneIDList
        s["boneToBoneID"] = boneToBoneID
        f.seek(jointDataOffset, 0)
        boneList = []
        for i in range(s["jointCount"]):
            b = {}
            b['i'] = i
            b['bone_id'] = 'bone_' + str(boneToBoneID.get(i, i))
            b['scale'] = list(struct.unpack(e+"3f", f.read(12)))
            b['parentID'], = struct.unpack(e+"i", f.read(4))
            rq = list(struct.unpack(e+"4f", f.read(16)))  # x,y,z,w
            b['q_wxyz'] = [rq[3], rq[0], rq[1], rq[2]]
            b['position'] = list(struct.unpack(e+"4f", f.read(16)))
            b['pos_xyz'] = b['position'][0:3]
            boneList.append(b)
        s["boneList"] = boneList
    return s

def calc_abs_skeleton(skel):
    def process(skel, idx):
        b  = skel['boneList'][idx]
        pb = skel['boneList'][b['parentID']]
        b['abs_q'] = quat_norm(quat_mul(pb['abs_q'], b['q_wxyz']))
        b['abs_p'] = vec_add(quat_rotate(pb['abs_q'], b['pos_xyz']), pb['abs_p'])
        for child in b.get('children', []):
            process(skel, child)

    for b in skel['boneList']:
        b['children'] = []
    for b in skel['boneList']:
        if b['parentID'] >= 0:
            skel['boneList'][b['parentID']]['children'].append(b['i'])
    for b in skel['boneList']:
        if b['parentID'] == -1:
            b['abs_q'] = b['q_wxyz']
            b['abs_p'] = b['pos_xyz']
            for child in b['children']:
                process(skel, child)
    return skel

def name_bones(skel, bones_dict):
    for b in skel['boneList']:
        idx = skel['boneToBoneID'].get(b['i'])
        if idx is not None and idx in bones_dict:
            b['bone_id'] = bones_dict[idx]
    return skel

def unpack_dxgi(f, stride, fmt, e='<'):
    fmt = fmt.split('DXGI_FORMAT_')[-1]
    parts = fmt.split('_')
    if len(parts) != 2:
        return f.read(stride)
    numtype = parts[1]
    nums = re.findall(r'\d+', parts[0])
    if not nums:
        return f.read(stride)
    bits  = int(nums[0])
    count = len(nums)
    size  = count * bits // 8
    if size != stride:
        return f.read(stride)
    if numtype == 'FLOAT':
        if bits == 32: return list(struct.unpack(e+str(count)+'f', f.read(stride)))
        if bits == 16: return list(struct.unpack(e+str(count)+'e', f.read(stride)))
    elif numtype == 'UINT':
        if bits == 32: return list(struct.unpack(e+str(count)+'I', f.read(stride)))
        if bits == 16: return list(struct.unpack(e+str(count)+'H', f.read(stride)))
        if bits == 8:  return list(struct.unpack(e+str(count)+'B', f.read(stride)))
    elif numtype == 'UNORM':
        if bits == 8:
            return [x/255.0 for x in struct.unpack(e+str(count)+'B', f.read(stride))]
        if bits == 16:
            return [x/65535.0 for x in struct.unpack(e+str(count)+'H', f.read(stride))]
    return f.read(stride)

def parseG1MG(chunk, e):
    s = {}
    with io.BytesIO(chunk) as f:
        s["magic"]    = f.read(4).decode("utf-8")
        s["version"], s["size"] = struct.unpack(e+"II", f.read(8))
        s["platform"] = f.read(4).decode("utf-8")
        f.read(4)
        f.read(24)
        s["sectionCount"], = struct.unpack(e+"I", f.read(4))

        sem_list  = ['POSITION','BLENDWEIGHT','BLENDINDICES','NORMAL','PSIZE','TEXCOORD',
                     'TANGENT','BINORMAL','TESSFACTOR','POSITIONT','COLOR','FOG','DEPTH','SAMPLE']
        dtype_map = {
            0x00:'R32_FLOAT',       0x01:'R32G32_FLOAT',        0x02:'R32G32B32_FLOAT',
            0x03:'R32G32B32A32_FLOAT', 0x05:'R8G8B8A8_UINT',   0x07:'R16G16B16A16_UINT',
            0x09:'R32G32B32A32_UINT',  0x0A:'R16G16_FLOAT',    0x0B:'R16G16B16A16_FLOAT',
            0x0D:'R8G8B8A8_UNORM', 0xFF:'UNKNOWN'
        }

        sections = []
        for _ in range(s["sectionCount"]):
            sec = {}
            sec['offset'] = f.tell()
            sec['magic'], sec['size'], sec['count'] = struct.unpack(e+"3I", f.read(12))

            if sec['magic'] == 0x00010001:
                sec['type'] = 'GEOMETRY_SOCKETS'
                for j in range(sec['count']):
                    f.read(32); f.read(32)
                tail_len = sec['size'] - (f.tell() - sec['offset'])
                f.read(tail_len)
                sec['data'] = []

            elif sec['magic'] == 0x00010002:
                sec['type'] = 'MATERIALS'
                mats = []
                for j in range(sec['count']):
                    _, tc, _, _ = struct.unpack(e+"4I", f.read(16))
                    textures = []
                    for k in range(tc):
                        t = {}
                        t['id'], t['layer'], t['type'], t['subtype'], t['tilemodex'], t['tilemodey'] = struct.unpack(e+"6H", f.read(12))
                        textures.append(t)
                    mats.append({'textures': textures})
                sec['data'] = mats

            elif sec['magic'] == 0x00010003:
                sec['type'] = 'SHADER_PARAMS'
                f.seek(sec['offset'] + sec['size'])
                sec['data'] = []

            elif sec['magic'] == 0x00010004:
                sec['type'] = 'VERTEX_BUFFERS'
                vbufs = []
                j = 0
                cur_off = 0
                while j < sec['count']:
                    buf = {}
                    buf['unknown1'], buf['stride'], buf['count'] = struct.unpack(e+"3I", f.read(12))
                    if s["version"] > 0x30303430:
                        buf['unknown2'], = struct.unpack(e+"I", f.read(4))
                    buf['offset'] = f.tell()
                    if buf['stride'] == 1:
                        cur_off = buf['offset']
                    f.seek(buf['stride'] * buf['count'], 1)
                    vbufs.append(buf)
                    j += 1
                    done = False
                    while not done:
                        flag, = struct.unpack(e+"I", f.read(4))
                        f.seek(-4, 1)
                        if flag == 0x80000000:
                            b2 = {}
                            b2['unknown1'], b2['stride'], b2['count'] = struct.unpack(e+"3I", f.read(12))
                            if s["version"] > 0x30303430:
                                b2['unknown2'], = struct.unpack(e+"I", f.read(4))
                            b2['offset'] = cur_off
                            cur_off += b2['stride'] * b2['count']
                            vbufs.append(b2)
                            j += 1
                        else:
                            done = True
                sec['data'] = vbufs

            elif sec['magic'] == 0x00010005:
                sec['type'] = 'VERTEX_ATTRIBUTES'
                attrs = []
                for j in range(sec['count']):
                    a = {}
                    lc, = struct.unpack(e+"I", f.read(4))
                    a['buffer_list'] = struct.unpack(e+str(lc)+"I", f.read(4*lc))
                    a['attr_count'], = struct.unpack(e+"I", f.read(4))
                    alist = []
                    for k in range(a['attr_count']):
                        attr = {}
                        attr['bufferID'], attr['offset'] = struct.unpack(e+"2H", f.read(4))
                        dt, attr['dummy_var'], sem, attr['layer'] = struct.unpack(e+"4B", f.read(4))
                        attr['dataType'] = dtype_map.get(dt, 'UNKNOWN')
                        attr['semantic'] = sem_list[sem] if sem < len(sem_list) else 'UNKNOWN'
                        alist.append(attr)
                    a['attributes_list'] = alist
                    attrs.append(a)
                sec['data'] = attrs

            elif sec['magic'] == 0x00010006:
                sec['type'] = 'JOINT_PALETTES'
                palettes = []
                for j in range(sec['count']):
                    jc, = struct.unpack(e+"I", f.read(4))
                    joints = []
                    for k in range(jc):
                        jt = {}
                        jt['G1MMIndex'], jt['physicsIndex'], jt['jointIndex'] = struct.unpack(e+"3I", f.read(12))
                        if jt['jointIndex'] > 0x80000000:
                            jt['physicsIndex'] ^= 0x80000000
                            jt['jointIndex']   ^= 0x80000000
                        joints.append(jt)
                    palettes.append({'joints': joints})
                sec['data'] = palettes

            elif sec['magic'] == 0x00010007:
                sec['type'] = 'INDEX_BUFFER'
                ibufs = []
                for j in range(sec['count']):
                    buf = {}
                    buf['count'], dt = struct.unpack(e+"II", f.read(8))
                    if s["version"] > 0x30303430:
                        f.read(4)
                    buf['dataType'] = "R{}_UINT".format(dt)
                    buf['stride']   = dt // 8
                    buf['offset']   = f.tell()
                    f.seek(buf['stride'] * buf['count'], 1)
                    if f.tell() % 4:
                        f.seek(4 - f.tell() % 4, 1)
                    ibufs.append(buf)
                sec['data'] = ibufs

            elif sec['magic'] == 0x00010008:
                sec['type'] = 'SUBMESH'
                subs = []
                for j in range(sec['count']):
                    sm = {}
                    (sm['submeshFlags'], sm['vertexBufferIndex'], sm['bonePaletteIndex'],
                     sm['boneIndex'], sm['unknown'], sm['shaderParamIndex'],
                     sm['materialIndex'], sm['indexBufferIndex'], sm['unknown2'],
                     sm['indexBufferPrimType'], sm['vertexBufferOffset'], sm['vertexCount'],
                     sm['indexBufferOffset'], sm['indexCount']) = struct.unpack(e+"14I", f.read(56))
                    subs.append(sm)
                sec['data'] = subs

            elif sec['magic'] == 0x00010009:
                sec['type'] = 'MESH_LOD'
                lods = []
                for j in range(sec['count']):
                    lb = {}
                    lb['LOD'], = struct.unpack(e+"I", f.read(4))
                    if s["version"] > 0x30303330:
                        lb['Group'], lb['GroupEntryIndex'] = struct.unpack(e+"2I", f.read(8))
                    lb['submeshCount1'], lb['submeshCount2'] = struct.unpack(e+"2I", f.read(8))
                    if s["version"] > 0x30303340:
                        f.read(16)
                    entries = []
                    for k in range(lb['submeshCount1'] + lb['submeshCount2']):
                        lod = {}
                        lod['name']    = f.read(16).replace(b'\x00', b'').decode("ASCII")
                        lod['clothID'], lod['unknown'], lod['NUNID'], lod['indexCount'] = struct.unpack(e+"2H2I", f.read(12))
                        if lod['indexCount'] > 0:
                            lod['indices'] = list(struct.unpack(e+"{}I".format(lod['indexCount']), f.read(4*lod['indexCount'])))
                        else:
                            lod['indices'] = []
                            f.seek(4, 1)
                        entries.append(lod)
                    lb['lod'] = entries
                    lods.append(lb)
                sec['data'] = lods

            else:
                sec['type'] = 'UNKNOWN'

            sections.append(sec)
            f.seek(sec['offset'] + sec['size'])

        s['sections'] = sections
    return s

def generate_fmts(meta, e='<'):
    vb    = next(x for x in meta['sections'] if x['type'] == 'VERTEX_BUFFERS')
    vbatt = next(x for x in meta['sections'] if x['type'] == 'VERTEX_ATTRIBUTES')
    ib    = next(x for x in meta['sections'] if x['type'] == 'INDEX_BUFFER')
    subvb = next(x for x in meta['sections'] if x['type'] == 'SUBMESH')

    vbsubs = {}
    for sm in subvb['data']:
        vbsubs.setdefault(sm['vertexBufferIndex'], []).append(sm)

    fmts = []
    for i, attr in enumerate(vbatt['data']):
        strides = [0]
        for bl in attr['buffer_list']:
            strides.append(vb['data'][bl]['stride'])
        total_stride = sum(strides)

        elems = []
        for j, a in enumerate(attr['attributes_list']):
            elems.append({
                'id': str(j),
                'SemanticName': a['semantic'],
                'SemanticIndex': str(a['layer']),
                'Format': a['dataType'],
                'AlignedByteOffset': str(a['offset'] + strides[a['bufferID']]),
            })

        prim = vbsubs.get(i, [{}])[0].get('indexBufferPrimType', 3)
        topo = {1:'pointlist', 3:'trianglelist', 4:'trianglestrip'}.get(prim, 'trianglelist')

        fmts.append({
            'stride': str(total_stride),
            'topology': topo,
            'format': 'DXGI_FORMAT_' + ib['data'][i]['dataType'],
            'elements': elems
        })
    return fmts

def read_vb(index, g1mg_stream, meta, fmts, e='<'):
    vb    = next(x for x in meta['sections'] if x['type'] == 'VERTEX_BUFFERS')
    vbatt = next(x for x in meta['sections'] if x['type'] == 'VERTEX_ATTRIBUTES')
    attr  = vbatt['data'][index]
    fmt   = fmts[index]
    stride    = int(fmt['stride'])
    buf_list  = attr['buffer_list']
    count     = vb['data'][buf_list[0]]['count']

    with io.BytesIO(g1mg_stream) as f:
        vb_stream = bytearray()
        for vi in range(count):
            for bl in buf_list:
                buf = vb['data'][bl]
                f.seek(buf['offset'] + buf['stride'] * (vi % buf['count']))
                vb_stream += f.read(buf['stride'])

    result = []
    for elem in fmt['elements']:
        eid = int(elem['id'])
        if eid < len(fmt['elements']) - 1:
            elem_stride = int(fmt['elements'][eid+1]['AlignedByteOffset']) - int(elem['AlignedByteOffset'])
        else:
            elem_stride = stride - int(elem['AlignedByteOffset'])
        buf_elem = {'SemanticName': elem['SemanticName'], 'SemanticIndex': elem['SemanticIndex'], 'Buffer': []}
        with io.BytesIO(vb_stream) as f:
            for vi in range(count):
                f.seek(vi * stride + int(elem['AlignedByteOffset']))
                buf_elem['Buffer'].append(unpack_dxgi(f, elem_stride, elem['Format'], e))
        result.append(buf_elem)
    return result

def read_ib(index, g1mg_stream, meta, e='<'):
    ib_sec = next(x for x in meta['sections'] if x['type'] == 'INDEX_BUFFER')
    buf    = ib_sec['data'][index]
    with io.BytesIO(g1mg_stream) as f:
        f.seek(buf['offset'])
        raw = f.read(buf['stride'] * buf['count'])
    fmt = 'H' if buf['stride'] == 2 else 'I'
    return list(struct.unpack(e + str(buf['count']) + fmt, raw))

def tstrip_to_list(strip):
    tris = []
    for i in range(len(strip) - 2):
        a, b, c = strip[i], strip[i+1], strip[i+2]
        if a == b or b == c or a == c:
            continue
        tris.append([a, b, c] if i % 2 == 0 else [a, c, b])
    return tris

def get_submesh(subindex, g1mg_stream, meta, skel, fmts, e='<'):
    subvb = next(x for x in meta['sections'] if x['type'] == 'SUBMESH')
    sm    = subvb['data'][subindex]
    fmt   = fmts[sm['vertexBufferIndex']]

    all_idx = read_ib(sm['indexBufferIndex'], g1mg_stream, meta, e)
    raw_ib  = all_idx[sm['indexBufferOffset'] : sm['indexBufferOffset'] + sm['indexCount']]
    faces   = tstrip_to_list(raw_ib) if fmt['topology'] == 'trianglestrip' \
              else [raw_ib[i:i+3] for i in range(0, len(raw_ib)-2, 3)]

    vb_data = read_vb(sm['vertexBufferIndex'], g1mg_stream, meta, fmts, e)

    used  = sorted(set(v for f in faces for v in f))
    remap = {old: new for new, old in enumerate(used)}
    new_vb = [{'SemanticName': el['SemanticName'], 'SemanticIndex': el['SemanticIndex'],
                'Buffer': [el['Buffer'][i] for i in used]} for el in vb_data]
    faces = [[remap[v] for v in f] for f in faces]

    vgmap = None
    try:
        palettes = next(x for x in meta['sections'] if x['type'] == 'JOINT_PALETTES')
        vgmap = {}
        for i, j in enumerate(palettes['data'][sm['bonePaletteIndex']]['joints']):
            if j['jointIndex'] < len(skel['boneList']):
                vgmap[skel['boneList'][j['jointIndex']]['bone_id']] = i * 3
    except Exception:
        pass

    return {'vb': new_vb, 'ib': faces, 'vgmap': vgmap}


# ================================================================
# READ G1M FILE -> (e, skel_data or None, g1mg_stream or None, mesh_meta or None)
# ================================================================

def read_g1m(path):
    e = '<'
    skel = None
    g1mg_stream = None
    meta = None

    with open(path, 'rb') as f:
        magic, = struct.unpack('>I', f.read(4))
        if magic == 0x5F4D3147:   e = '<'
        elif magic == 0x47314D5F: e = '>'
        else: return None, None, None, None

        f.read(8)
        start_off, _, chunk_count = struct.unpack(e+'3I', f.read(12))
        f.seek(start_off)
        have_skel = False

        for _ in range(chunk_count):
            cs  = f.tell()
            cm  = f.read(4).decode('utf-8')
            f.read(4)
            csz, = struct.unpack(e+'I', f.read(4))

            if cm in ('G1MS', 'SM1G') and not have_skel:
                f.seek(cs)
                skel = parseG1MS(f.read(csz), e)
                oid = path[:-4] + 'Oid.bin'
                if os.path.exists(oid):
                    skel = name_bones(skel, binary_oid_to_dict(oid))
                if skel['jointCount'] > 1 and not skel['boneList'][0]['parentID'] < -200000000:
                    skel = calc_abs_skeleton(skel)
                have_skel = True

            elif cm in ('G1MG', 'GM1G'):
                f.seek(cs)
                g1mg_stream = f.read(csz)
                meta = parseG1MG(g1mg_stream, e)

            else:
                f.seek(cs + csz)

    return e, skel, g1mg_stream, meta


# ================================================================
# BLENDER
# ================================================================

def build_armature(skel, name, col):
    arm = bpy.data.armatures.new(name + '')
    obj = bpy.data.objects.new(name + '', arm)
    col.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    bpy.ops.object.mode_set(mode='EDIT')
    for bd in skel['boneList']:
        try:
            eb = arm.edit_bones.new(bd['bone_id'])
            p  = bd.get('abs_p', bd['pos_xyz'])
            eb.head = (p[0], p[1], p[2])
            eb.tail = (p[0], p[1], p[2] + 0.05)
            if 0 <= bd['parentID'] < len(skel['boneList']):
                pname = skel['boneList'][bd['parentID']]['bone_id']
                if pname in arm.edit_bones:
                    eb.parent = arm.edit_bones[pname]
        except Exception as ex:
            print("  Bone {}: {}".format(bd.get('bone_id', '?'), ex))
    bpy.ops.object.mode_set(mode='OBJECT')
    obj.rotation_euler[0] = math.pi / 2
    return obj

def build_mesh(submesh, label, arm_obj, col):
    sem = {el['SemanticName']+'_'+str(el['SemanticIndex']): el for el in submesh['vb']}

    pos_key = next((k for k in sem if k.startswith('POSITION')), None)
    if not pos_key:
        return None

    verts       = [v[0:3] for v in sem[pos_key]['Buffer']]
    valid_faces = [f for f in submesh['ib'] if len(set(f)) == 3]

    mdata = bpy.data.meshes.new(label)
    obj   = bpy.data.objects.new(label, mdata)
    col.objects.link(obj)
    mdata.from_pydata(verts, [], valid_faces)
    mdata.update()

    for key, el in sem.items():
        if not key.startswith('TEXCOORD'):
            continue
        uv_name  = 'UVMap' if el['SemanticIndex'] == '0' else 'UV_' + el['SemanticIndex']
        uv_layer = mdata.uv_layers.new(name=uv_name)
        uv_buf   = el['Buffer']
        for poly in mdata.polygons:
            for li, vi in zip(poly.loop_indices, poly.vertices):
                if vi < len(uv_buf) and uv_buf[vi]:
                    uv_layer.data[li].uv = (uv_buf[vi][0], 1.0 - uv_buf[vi][1])

    vgmap = submesh.get('vgmap')
    if vgmap:
        rev    = {v: k for k, v in vgmap.items()}
        blidx  = sorted([x for x in submesh['vb'] if x['SemanticName'] == 'BLENDINDICES'],
                        key=lambda x: int(x['SemanticIndex']))
        blwt   = sorted([x for x in submesh['vb'] if x['SemanticName'] in ('BLENDWEIGHT','BLENDWEIGHTS')],
                        key=lambda x: int(x['SemanticIndex']))
        for layer in range(min(len(blidx), len(blwt))):
            idx_buf = blidx[layer]['Buffer']
            wt_buf  = blwt[layer]['Buffer']
            for vi in range(len(verts)):
                idxs = idx_buf[vi] if vi < len(idx_buf) else []
                wts  = wt_buf[vi]  if vi < len(wt_buf)  else []
                for i, raw in enumerate(idxs):
                    w = wts[i] if i < len(wts) else 0.0
                    if w <= 0.0: continue
                    bname = rev.get(raw)
                    if not bname: continue
                    if bname not in obj.vertex_groups:
                        obj.vertex_groups.new(name=bname)
                    obj.vertex_groups[bname].add([vi], w, 'ADD')

    nml_key = next((k for k in sem if k.startswith('NORMAL')), None)
    if nml_key:
        normals = [sem[nml_key]['Buffer'][i][0:3] for i in range(len(verts))]
        try:
            mdata.use_auto_smooth = True
            mdata.normals_split_custom_set_from_vertices(normals)
        except Exception:
            pass

    if arm_obj:
        obj.parent = arm_obj
        mod = obj.modifiers.new('Armature', 'ARMATURE')
        mod.object = arm_obj
        mod.use_vertex_groups = True
    else:
        obj.rotation_euler[0] = math.pi / 2

    return obj


# ================================================================
# ENTRY POINT
# ================================================================

def import_single(path):
    base = os.path.splitext(os.path.basename(path))[0]
    e, skel, g1mg_stream, meta = read_g1m(path)

    col = bpy.data.collections.new(base)
    bpy.context.scene.collection.children.link(col)

    arm_obj = None
    if skel and skel['jointCount'] > 0:
        print("  {}: skeleton {} bones".format(base, skel['jointCount']))
        arm_obj = build_armature(skel, base, col)

    if not meta:
        return

    subvb   = next((x for x in meta['sections'] if x['type'] == 'SUBMESH'), None)
    lod_sec = next((x for x in meta['sections'] if x['type'] == 'MESH_LOD'), None)
    if not subvb or not lod_sec:
        return

    lod_flat = [lod for grp in lod_sec['data'] for lod in grp['lod']]
    fmts     = generate_fmts(meta, e)
    total    = 0

    for si in range(len(subvb['data'])):
        lod = next((x for x in lod_flat if si in x['indices']), None)
        if lod and lod['clothID'] != 0:
            continue
        try:
            sm = get_submesh(si, g1mg_stream, meta, skel, fmts, e)
            if sm['ib']:
                label = '{}_m{:03d}'.format(base, si)
                if build_mesh(sm, label, arm_obj, col):
                    total += 1
        except Exception as ex:
            print("  Skipped {}_m{}: {}".format(base, si, ex))

    print("  {}: {} meshes".format(base, total))

def import_folder(folder):
    folder    = os.path.abspath(folder)
    g1m_files = sorted(glob.glob(os.path.join(folder, '*.g1m')))

    if not g1m_files:
        print("No .g1m files found in", folder)
        return

    print("Found {} g1m file(s) in {}".format(len(g1m_files), folder))

    old_dir = os.getcwd()
    os.chdir(folder)
    try:
        for path in g1m_files:
            import_single(path)
    finally:
        os.chdir(old_dir)


import_folder(G1M_FOLDER)
