import sys
import os
import time

from svgpathtools import *
import numpy as np
# import matplotlib.pyplot as plt
import triangle as tr
import pymeshlab
# import openmesh as om

assert(len(sys.argv) >= 3)
name = sys.argv[1]

triResolution = 1
resolution = 2
if len(sys.argv) >= 3:
    resolution = float(sys.argv[2])

seamWidth = 1
if len(sys.argv) >= 4:
    seamWidth = float(sys.argv[3])

lastProcessingTime = -1


def preprocess(file_name, reso, seam_w, tri_reso):
    gap = reso     # mm

    file_name_new = ".".join(file_name.split('.')[:-1]) + "_ext.svg"
    with open(file_name) as ifile:
        content = ifile.read()

    while True:
        g0 = content.find('<g')
        g1 = content.find('/g>') + 3

        if g0 == -1 or g1 == -1:
            print('*** g0 not found or g1 not found')
            break
        else:
            print('*** g0 and g1 found')

        content = content[:g0] + content[g1:]

    with open(file_name_new, 'w') as ofile:
        ofile.write(content)

    paths, attributes, svg_attributes = svg2paths2(file_name)

    vs = []
    es = []
    holes = []

    spanMax = -float('inf')
    xMin = yMin = float('inf')
    xMax = yMax = -float('inf')

    pathss = []
    for p in paths:
        try:
            xm, xM, ym, yM = p.bbox()
            if p.length() > gap * 3:
                pathss.append(p)
        except:
            continue
    paths = pathss[:]

    for p in paths:
        xm, xM, ym, yM = p.bbox()
        xMin = min(xMin, xm)
        yMin = min(yMin, ym)
        xMax = max(xMax, xM)
        yMax = max(yMax, yM)
        spanMax = max(xM - xm, spanMax, yM - ym)
    centerX = (xMin + xMax) / 2
    centerY = (yMin + yMax) / 2

    scale = 200. / spanMax

    for i in range(len(paths)):
        paths[i] = paths[i].translated(complex(-centerX, -centerY))

    p_closed = []
    points_closed = []

    print('*** Converting curves to regions...')
    for p in paths:
        length = p.length()
        nSegs = int(length / tri_reso)

        points = [p.point(n / nSegs) for n in range(nSegs)]
        if p.isclosed():
            points.append(p.point(1))
        points = [[point.real * scale, point.imag * scale] for point in points]

        if p.isclosed():
            points_closed.append(points)
            p_closed.append(p)
            lenVs = len(vs)
            for i in range(len(points) - 1):
                es.append([lenVs + i, lenVs + i + 1])
            es.append([lenVs + len(points) - 1, lenVs])
            vs += points
        else:
            ps = np.array(points)
            pointsOffset = []
            for i in range(ps.shape[0]):
                point = ps[i]
                if i == 0:
                    pointPrev = ps[i]
                else:
                    pointPrev = ps[i - 1]
                if i == ps.shape[0] - 1:
                    pointNext = ps[i]
                else:
                    pointNext = ps[i + 1]

                vecPrev = point - pointPrev
                vecNext = pointNext - point
                vecMean = (vecPrev + vecNext) / 2
                normal = np.array([-vecMean[1], vecMean[0]])
                norm = np.sqrt(np.sum(normal * normal))
                normal /= norm
                pointOffset = point - normal * seam_w
                pointHole = (pointOffset + point) / 2
                if i == len(ps) // 2:
                    holes.append([pointHole[0], pointHole[1]])
                pointsOffset.append(pointOffset)

            points[0] = (points[0] + pointsOffset[0]) / 2
            points[-1] = (points[-1] + pointsOffset[-1]) / 2
            pointsOffset = pointsOffset[1:-1]

            id0 = len(vs)
            for i in range(len(points)):
                if i != len(points) - 1:
                    es.append([len(vs), len(vs) + 1])
                vs.append(points[i])

            for i in range(len(pointsOffset)):
                es.append([len(vs) - 1, len(vs)])
                if i == len(pointsOffset) - 1:
                    es.append([len(vs), id0])
                vs.append(pointsOffset[::-1][i])

    areas = [abs(p.area()) for p in p_closed]
    print("*** Areas:", areas)
    iMaxArea = areas.index(max(areas))
    for i in range(len(p_closed)):
        if i != iMaxArea:
            holes.append(np.mean(points_closed[i], axis=0))

    print('*** Triangulating...')
    A = dict(vertices=vs, segments=es, holes=holes)
    B = tr.triangulate(A, 'p')
    # B = A
    # tr.compare(plt, A, B)
    # plt.show()

    vs = B['vertices']
    vs = np.hstack([vs, np.zeros([vs.shape[0], 1])])
    faces = B['triangles']
    fs = np.array(faces, dtype=np.int32)

    ms = pymeshlab.MeshSet()
    mesh = pymeshlab.Mesh(vs, fs)
    ms.add_mesh(mesh)

    print("*** Removing low quality faces...")
    ms.remove_isolated_pieces_wrt_face_num()
    ms.remove_isolated_pieces_wrt_diameter()

    print("*** Merging close vertices...")
    ms.merge_close_vertices()
    print("*** Isotropic explicit remeshing...")
    ms.remeshing_isotropic_explicit_remeshing(targetlen=gap, maxsurfdist=gap, iterations=20)
    ms.invert_faces_orientation()

    ms.save_current_mesh("./data/out.obj")
    # save time stamp
    with open('./data/timestamp', 'w') as ofile:
        ofile.write(str(int(time.time())))

    print("### py: Done preprocessing...")


def is_file_changed(file_name):
    global lastProcessingTime
    mtime = os.path.getmtime(file_name)
    if mtime - lastProcessingTime > 1:
        print('*** py: SVG file change detected...')
        lastProcessingTime = mtime
        return True
    return False


while True:
    time.sleep(.5)
    if is_file_changed(name):
        print("*** Preprocess with resolution: {}, seamWidth: {}, triResolution: {}".format(resolution, seamWidth, triResolution))
        preprocess(name, resolution, seamWidth, triResolution)



# vs = ms.current_mesh().vertex_matrix()
# fs = ms.current_mesh().face_matrix()
#
# mesh = om.TriMesh()
# vertices = [mesh.add_vertex(v) for v in vs]
# faces = [mesh.add_face([vertices[f[0]], vertices[f[1]], vertices[f[2]]]) for f in fs]
#
# halfedges = [h for h in mesh.halfedges()]
#
# print(len(list(mesh.halfedges())))
# for i in range(800):
#     mesh.collapse(next(mesh.halfedges()))
# print(len(list(mesh.halfedges())))

# vs = list(mesh.vertices())
# fs = list(mesh.faces())
#
# ms = pymeshlab.MeshSet()
# mesh = pymeshlab.Mesh(vs, fs)
# ms.add_mesh(mesh)
#
# ms.save_current_mesh(sys.argv[2] + "data/out.obj")
